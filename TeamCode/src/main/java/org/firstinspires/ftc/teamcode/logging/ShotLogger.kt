package org.firstinspires.ftc.teamcode.logging

import org.firstinspires.ftc.teamcode.RobotConstants
import org.firstinspires.ftc.teamcode.game.ShotFeatures
import java.io.File
import kotlin.math.hypot

/**
 * Saves one row per shot to a .csv file, to train the shot model (flywheel speed and hood angle
 * from distance, speed and so on). BIOBUZZ_GUIDE.md, section 10, explains the columns.
 *
 * How it works:
 *  1. Every loop the OpMode gives us a LoopData: everything we know about the robot and the shooter.
 *     We keep the last half second of them.
 *  2. The ShotDetector finds the moment a ball leaves (the flywheel suddenly slows down). The row uses
 *     the loop just BEFORE that moment, so it describes the robot as it was when the ball left.
 *  3. The shot waits until someone presses a label button (went in, missed, ...). Labels go to the
 *     oldest waiting shot first. Then the row is written.
 *
 * Nothing here can stop the robot: if the file cannot be written, logging just stops.
 */
class ShotLogger(folder: File, opModeName: String, private val labelTimeoutSeconds: Double) {

    /** Everything about one loop that a shot row needs. ShotLogData.kt fills it in. */
    data class LoopData(
        val seconds: Double,                 // since START
        val loopMs: Double,                  // how long the last loop took
        val alliance: String,
        val cell: String,                    // the CELL we aim at
        val mode: String,                    // TELEOP, SWEEP or TEST
        val x: Double, val y: Double, val headingDegrees: Double,
        val vx: Double, val vy: Double,      // robot velocity on the field (in/s)
        val targetX: Double, val targetY: Double,   // the CELL opening
        val features: ShotFeatures,
        val turretCommandDegrees: Double,
        val turretEstimateDegrees: Double,
        val turretSettled: Boolean,
        val leadDegrees: Double,             // how far the aim is moved off the target for shooting on the move
        val flightSeconds: Double,           // flight time used for that (0 = no lead)
        val aimDistanceInches: Double,       // distance to the point we aim at (the shot table uses this)
        val distanceTrimInches: Double,
        val turretTrimDegrees: Double,
        val manualMode: Boolean,
        val tableRpm: Double,                // what the shot table asked for
        val tableHood: Double,
        val rpmOffset: Double,               // added on top in SWEEP mode
        val hoodOffset: Double,
        val targetRpm: Double,               // what the flywheel was told
        val rpm: Double,                     // what the flywheel really did
        val hoodPosition: Double,            // servo position sent to the hood
        val hoodDegrees: Double,             // hood angle (NaN until the hood is calibrated)
        val ready: Boolean,                  // was the shooter READY?
        val forced: Boolean,                 // did the operator shoot anyway (cross)?
        val feeding: Boolean,                // were we pushing POLLEN into the flywheel this loop?
        val batteryVolts: Double,
        val secondsSinceTagFix: Double,      // since the webcam last fixed our position
    )

    /** A shot waiting for its label. */
    private class Shot(
        val number: Int,
        val launch: LoopData,
        val detectedSeconds: Double,
        val shotInBurst: Int,
        val secondsSincePreviousShot: Double,
        val acceleration: Double,
        val hoodMoved: Double,
    ) {
        var launchRpm = launch.rpm
        var dipRpm = 0.0
    }

    private val shotFile = CsvFile(File(folder, "shots"), "shots_" + fileSafe(opModeName), SHOT_COLUMNS)
    private val traceFile: CsvFile? =
        if (RobotConstants.SHOT_LOG_TRACE) CsvFile(File(folder, "shots"), "trace_" + fileSafe(opModeName), TRACE_COLUMNS) else null
    private val runName = shotFile.file?.nameWithoutExtension ?: "unknown"
    private val opMode = opModeName
    private val detector = ShotDetector()
    private val recentLoops = ArrayDeque<LoopData>()
    private val waiting = ArrayList<Shot>()
    private var newestShot: Shot? = null
    private var lastLabeledShot: Shot? = null
    private var lastLaunchSeconds = -1000.0
    private var shotInBurst = 0
    private var lastTraceFlushSeconds = 0.0

    /** How many shots were found so far. */
    var shotsFound = 0
        private set

    /** How many rows are in the file. */
    var rowsWritten = 0
        private set

    /** How many shots are waiting for a label. */
    val waitingForLabel: Int
        get() = waiting.size

    /** False if the file could not be opened (then nothing is logged). */
    fun isWorking(): Boolean = shotFile.isWorking()

    /** The file name, to show on the Driver Station (or why nothing is being saved). */
    fun fileName(): String {
        val file = shotFile.file ?: return "LOG OFF (could not open the file)"
        if (!shotFile.isWorking()) return "LOG OFF (writing failed: is the storage full?)"
        return file.name
    }

    /** How fast the last shot left (RPM) and how much the flywheel slowed down, for the screen. */
    fun lastShotSummary(): String {
        val shot = newestShot ?: return "no shots yet"
        return "#%d: %.0f in, toward %+.0f, left %+.0f in/s -> %.0f rpm, dip %.0f".format(
            shot.number, shot.launch.features.distanceInches, shot.launch.features.towardSpeed,
            shot.launch.features.leftSpeed, shot.launchRpm, shot.dipRpm
        )
    }

    /** Call every loop. Returns true on the loop a new shot is found. */
    fun update(data: LoopData): Boolean {
        recentLoops.addLast(data)
        while (recentLoops.first().seconds < data.seconds - 0.5) {
            recentLoops.removeFirst()
        }

        // Feeding the next ball ends the chance to fix the last label: a later press is about the new ball.
        if (data.feeding) {
            lastLabeledShot = null
        }
        val newShot = detector.update(data.seconds, data.feeding, data.targetRpm, data.rpm)
        if (newShot) {
            addShot(data.seconds)
        }
        // Keep the newest shot's dip up to date until the flywheel speeds up again.
        if (detector.inDip) {
            newestShot?.dipRpm = detector.dipRpm
        }

        // Shots nobody labeled in time are saved as UNLABELED.
        while (waiting.isNotEmpty() && data.seconds - waiting.first().detectedSeconds > labelTimeoutSeconds) {
            writeRow(waiting.removeAt(0), UNLABELED, data.seconds)
        }

        writeTrace(data, newShot)
        return newShot
    }

    /** Gives the oldest waiting shot its label (IN, MISSED, SHORT, ...). Returns false if no shot is waiting. */
    fun label(result: String, nowSeconds: Double): Boolean {
        if (waiting.isEmpty()) return false
        val shot = waiting.removeAt(0)
        writeRow(shot, result, nowSeconds)
        lastLabeledShot = shot
        return true
    }

    /**
     * Changes the label of the last labeled shot (for a wrong button press), until the next ball is fed.
     * The file gets a second row with the same shot number: when reading the file, keep the LAST row of
     * each shot. Returns false if there is nothing to change.
     */
    fun relabelLastShot(result: String, nowSeconds: Double): Boolean {
        val shot = lastLabeledShot ?: return false
        writeRow(shot, result, nowSeconds)
        return true
    }

    /** Call when the OpMode stops: saves the shots still waiting (as UNLABELED) and closes the files. */
    fun close(nowSeconds: Double) {
        for (shot in waiting) {
            writeRow(shot, UNLABELED, nowSeconds)
        }
        waiting.clear()
        shotFile.close()
        traceFile?.close()
    }

    private fun addShot(nowSeconds: Double) {
        shotsFound += 1
        // The loop just before the flywheel started slowing down: the moment the ball left.
        val launch = recentLoops.minByOrNull { kotlin.math.abs(it.seconds - detector.launchSeconds) } ?: recentLoops.last()
        val sincePrevious = launch.seconds - lastLaunchSeconds
        shotInBurst = if (sincePrevious > 1.0) 1 else shotInBurst + 1
        lastLaunchSeconds = launch.seconds

        // How fast the robot's speed was changing (in/s per second) over the 0.1 s before the shot.
        val before = recentLoops.firstOrNull { it.seconds >= launch.seconds - 0.1 } ?: launch
        val dt = launch.seconds - before.seconds
        val acceleration = if (dt > 0.02) (hypot(launch.vx, launch.vy) - hypot(before.vx, before.vy)) / dt else 0.0
        // How much the hood moved in the 0.3 s before the shot (a moving hood is not where we think).
        var lowestHood = launch.hoodPosition
        var highestHood = launch.hoodPosition
        for (loop in recentLoops) {
            if (loop.seconds < launch.seconds - 0.3 || loop.seconds > launch.seconds) continue
            lowestHood = minOf(lowestHood, loop.hoodPosition)
            highestHood = maxOf(highestHood, loop.hoodPosition)
        }

        val shot = Shot(
            number = shotsFound,
            launch = launch,
            detectedSeconds = nowSeconds,
            shotInBurst = shotInBurst,
            secondsSincePreviousShot = if (shotsFound == 1) Double.NaN else sincePrevious,
            acceleration = acceleration,
            hoodMoved = highestHood - lowestHood,
        )
        shot.launchRpm = detector.launchRpm
        shot.dipRpm = detector.dipRpm
        waiting.add(shot)
        newestShot = shot
    }

    private fun writeRow(shot: Shot, result: String, nowSeconds: Double) {
        val d = shot.launch
        val f = d.features
        shotFile.addRow(
            listOf(
                runName, opMode, RobotConstants.SHOOTER_CONFIG_TAG, shot.number, d.seconds, d.alliance, d.cell, d.mode,
                f.distanceInches, f.towardSpeed, f.leftSpeed, f.turnDegreesPerSecond, f.offAxisDegrees,
                d.x, d.y, d.headingDegrees, d.vx, d.vy, shot.acceleration, d.targetX, d.targetY,
                d.turretCommandDegrees, d.turretEstimateDegrees, d.turretSettled, d.leadDegrees, d.flightSeconds,
                d.aimDistanceInches, d.distanceTrimInches, d.turretTrimDegrees, d.manualMode,
                d.tableRpm, d.tableHood, d.rpmOffset, d.hoodOffset, d.targetRpm, shot.launchRpm,
                shot.launchRpm - d.targetRpm, d.hoodPosition, d.hoodDegrees, shot.hoodMoved, shot.dipRpm,
                d.ready, d.forced, shot.shotInBurst, shot.secondsSincePreviousShot,
                (shot.detectedSeconds - d.seconds) * 1000.0, d.batteryVolts, d.loopMs, d.secondsSinceTagFix,
                result, nowSeconds - shot.detectedSeconds,
            )
        )
        shotFile.flush()   // shots are rare, so save each one right away
        if (shotFile.isWorking()) {
            rowsWritten += 1
        }
    }

    private fun writeTrace(d: LoopData, newShot: Boolean) {
        val trace = traceFile ?: return
        trace.addRow(
            listOf(
                d.seconds, d.loopMs, d.feeding, d.targetRpm, d.rpm, detector.inDip, if (newShot) shotsFound else 0,
                d.hoodPosition, d.features.distanceInches, d.features.towardSpeed, d.features.leftSpeed,
                d.features.turnDegreesPerSecond, d.batteryVolts,
            )
        )
        if (d.seconds - lastTraceFlushSeconds > 0.5) {
            trace.flush()
            lastTraceFlushSeconds = d.seconds
        }
    }

    companion object {
        // Labels. Teleop only uses IN and MISSED; the Shot Collector uses the rest too.
        const val IN = "IN"
        const val MISSED = "MISSED"
        const val SHORT = "SHORT"
        const val LONG = "LONG"
        const val WIDE_LEFT = "WIDE_LEFT"
        const val WIDE_RIGHT = "WIDE_RIGHT"
        const val OTHER = "OTHER"
        const val NOT_A_SHOT = "NOT_A_SHOT"
        const val UNLABELED = "UNLABELED"

        val SHOT_COLUMNS = listOf(
            "run", "opmode", "config", "shot", "t_s", "alliance", "cell", "mode",
            "distance_in", "v_toward_in_s", "v_left_in_s", "turn_deg_s", "off_axis_deg",
            "x_in", "y_in", "heading_deg", "vx_in_s", "vy_in_s", "accel_in_s2", "target_x_in", "target_y_in",
            "turret_cmd_deg", "turret_est_deg", "turret_settled", "lead_deg", "flight_s",
            "aim_distance_in", "distance_trim_in", "turret_trim_deg", "manual_mode",
            "table_rpm", "table_hood", "rpm_offset", "hood_offset", "target_rpm", "launch_rpm",
            "rpm_error", "hood_pos", "hood_deg", "hood_moved", "dip_rpm",
            "ready", "forced", "shot_in_burst", "since_prev_shot_s",
            "detect_delay_ms", "battery_v", "loop_ms", "since_tag_fix_s",
            "result", "label_delay_s",
        )

        val TRACE_COLUMNS = listOf(
            "t_s", "loop_ms", "feeding", "target_rpm", "rpm", "in_dip", "shot",
            "hood_pos", "distance_in", "v_toward_in_s", "v_left_in_s", "turn_deg_s", "battery_v",
        )

        /** "TeleOp Red" -> "TeleOp-Red": file names should not have spaces. */
        private fun fileSafe(name: String): String {
            return name.replace(Regex("[^A-Za-z0-9_-]"), "-")
        }
    }
}
