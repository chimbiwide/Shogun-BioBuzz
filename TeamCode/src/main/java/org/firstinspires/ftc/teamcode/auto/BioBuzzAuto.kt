package org.firstinspires.ftc.teamcode.auto

import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.api.Paths
import com.pedropathing.follower.Follower
import com.pedropathing.math.Pose
import com.pedropathing.utils.Angle
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.RobotConstants
import org.firstinspires.ftc.teamcode.game.Alliance
import org.firstinspires.ftc.teamcode.game.Field
import org.firstinspires.ftc.teamcode.game.HiveTracker
import org.firstinspires.ftc.teamcode.game.PoseStorage
import org.firstinspires.ftc.teamcode.game.Relocalizer
import org.firstinspires.ftc.teamcode.pedro.Constants
import org.firstinspires.ftc.teamcode.tools.Intake
import org.firstinspires.ftc.teamcode.tools.Shooter
import org.firstinspires.ftc.teamcode.tools.Transfer
import org.firstinspires.ftc.teamcode.tools.Webcam
import kotlin.math.abs
import kotlin.math.hypot

/** The two autonomous plans. Pick the one that does not get in our alliance partner's way. */
enum class Routine {
    /** Start next to our GARDEN, on the wall our up CELL opens toward (RED: audience wall, BLUE: far wall). */
    GARDEN,
    /** Start on the other wall (RED: far wall, BLUE: audience wall) and drive around the HIVE. */
    GO_AROUND
}

/*
 * BIOBUZZ autonomous. Pick one of the four OpModes at the bottom of this file.
 *
 * Important: a CELL's opening faces the wall that CELL points at, so we can only score from that
 * side of the HIVE. Our CELL that starts up opens toward the audience wall (for BLUE: the far wall).
 *
 * GARDEN:    start on that wall (RED: audience wall, BLUE: far wall), facing the HIVE. Shoot the
 *            4 pre-loaded POLLEN, pick up our GARDEN's 4 POLLEN, shoot them, park in the LOADING ZONE.
 * GO_AROUND: start on the other wall (RED: far wall, BLUE: audience wall). Drive around the HIVE along
 *            our alliance wall, shoot the 4 pre-loaded POLLEN, park.
 *            Pick this one when our partner wants the GARDEN start.
 *
 * During INIT, the driver can change two things with gamepad 1 (look at the Driver Station screen):
 *   dpad up/down ...... wait this many seconds before starting (to stay out of our partner's way)
 *   dpad left ......... park in the half of the LOADING ZONE on your LEFT (the corner half)
 *   dpad right ........ park in the half on your RIGHT (the middle half). Our partner takes the other half.
 *
 * At AUTO_PARK_START_SECONDS the robot drops whatever it is doing and parks (LEAVE + PARK = 8 points).
 *
 * Read runGarden() and runGoAround() first: they read like a list of steps.
 * The helper functions below them do the work. (SimpleAuto.kt has its own copy of some of them.)
 */
abstract class BioBuzzAuto(private val alliance: Alliance, private val routine: Routine) : LinearOpMode() {
    private lateinit var hubs: List<LynxModule>
    private lateinit var follower: Follower
    private lateinit var intake: Intake
    private lateinit var transfer: Transfer
    private lateinit var shooter: Shooter
    private lateinit var hive: HiveTracker
    private var webcam: Webcam? = null
    private var pinpoint: GoBildaPinpointDriver? = null
    private val relocalizer = Relocalizer()
    // Never use "runtime" for match time: it starts counting at INIT, not at START.
    private val matchTimer = ElapsedTime()
    private val timeSinceTagsSeen = ElapsedTime()
    private var step = "waiting for start"

    // Settings the driver can change during INIT.
    private var startDelaySeconds = 0.0
    private var parkInCorner = (routine == Routine.GO_AROUND)

    override fun runOpMode() {
        // Telemetry also goes to Panels, but only when Panels is turned on (see RobotConstants.USE_PANELS).
        if (RobotConstants.USE_PANELS) {
            telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry)
        }
        // Read all the motor encoders at once, one time per loop, so the loop runs faster.
        // MANUAL means we throw away the old readings ourselves: updateRobot() does that every loop.
        hubs = hardwareMap.getAll(LynxModule::class.java)
        for (hub in hubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }

        // If this line crashes, Pedro is not tuned yet: see pedro/Constants.java.
        // Until then, use the "Simple" autonomous (SimpleAuto.kt).
        follower = Constants.create(hardwareMap)
        intake = Intake(hardwareMap)
        transfer = Transfer(hardwareMap)
        shooter = Shooter(hardwareMap)
        hive = HiveTracker(alliance)
        if (RobotConstants.USE_WEBCAM) {
            webcam = Webcam(hardwareMap)
        }
        // The same Pinpoint the follower uses, to correct only x and y when the webcam fixes our position.
        pinpoint = hardwareMap.getAll(GoBildaPinpointDriver::class.java).firstOrNull()

        val startPose = place(if (routine == Routine.GARDEN) AutoPoses.AUDIENCE_START else AutoPoses.FAR_START)
        follower.setPose(startPose)
        // Point the turret and hood, and make sure the transfer is stopped, once, now.
        // Nothing may move after INIT is done (rule G304.H), so the INIT loop below only reads
        // buttons and shows telemetry.
        shooter.aimAt(startPose, hive.target())
        transfer.stop()

        while (opModeInInit()) {
            chooseSettings()
            val camera = webcam
            if (camera != null) {
                camera.applyCameraSettings()   // short exposure, once the camera is streaming
                camera.update()
                hive.updateFromCamera(camera.findUpCell(alliance))
            }
            showInitTelemetry()
            sleep(20)   // OK here: before START there is nothing to keep updating
        }
        if (isStopRequested) return

        matchTimer.reset()
        // Set the start position again: the Pinpoint can miss a position set while it is still calibrating.
        follower.setPose(startPose)
        follower.update()

        try {
            if (routine == Routine.GARDEN) {
                runGarden()
            } else {
                runGoAround()
            }

            // Done. Hold the parking spot until autonomous ends.
            step = "parked"
            while (opModeIsActive()) {
                updateRobot()
            }
        } finally {
            // Pedro remembers a path's slow speed until that path ends. If autonomous was stopped
            // in the middle of a slow path, this clears it, so the next run is not slow.
            // (The robot app closes the webcam by itself.)
            follower.stop()
        }
    }

    private fun runGarden() {
        shooter.isOn = true                         // start the flywheel now, so it is up to speed sooner
        waitSeconds(startDelaySeconds)
        val shotPreloads = shootAll()               // the 4 POLLEN we started with

        driveTo(AutoPoses.LEAVE_AUDIENCE_WALL)      // off the wall first, so there is room to turn
        if (shotPreloads) {
            // Only pick up more POLLEN when the robot is empty: never more than 4 (rule G407).
            driveTo(AutoPoses.GARDEN_APPROACH)
            collectGarden()                         // our GARDEN's 4 POLLEN
            driveTo(AutoPoses.GARDEN_BACK_OFF)
        }
        goToShootingSpot()
        shootAll()

        park()
    }

    private fun runGoAround() {
        waitSeconds(startDelaySeconds)
        driveTo(AutoPoses.LEAVE_FAR_WALL)
        goToShootingSpot()                          // drives around the HIVE
        shootAll()                                  // the 4 POLLEN we started with

        park()
    }

    // ------------------------------------------------------------------------------------------
    // Helpers
    // ------------------------------------------------------------------------------------------

    /** AutoPoses are written for RED. This gives the matching pose for our alliance. */
    private fun place(redPose: Pose): Pose {
        return Field.forAlliance(redPose, alliance)
    }

    /** True if the robot is on the audience side of the HIVE, as RED sees it (AutoPoses are RED poses). */
    private fun onAudienceSide(): Boolean {
        val asRed = Field.forAlliance(follower.pose(), alliance)
        return asRed.y() < Field.HIVE_Y
    }

    /** Everything that must happen every loop. Every waiting loop below calls this. */
    private fun updateRobot() {
        // Throw away last loop's motor readings (MANUAL bulk caching, see runOpMode).
        for (hub in hubs) {
            hub.clearBulkCache()
        }
        follower.update()
        val pose = follower.pose()

        // The webcam. Our tags face down, leaning toward the side our up CELL opens to. From our
        // autonomous shooting spots it sees them while that CELL opens toward us. After a tip they
        // lean away, so the webcam cannot see the tip itself, only that our tags are gone.
        // (It only changes the target if USE_CAMERA_TO_FIND_UP_CELL is on.)
        val camera = webcam
        if (camera != null) {
            camera.applyCameraSettings()   // does nothing once the exposure is set
            camera.update()
            if (camera.seesOurTags(alliance)) {
                timeSinceTagsSeen.reset()
            }
            hive.updateFromCamera(camera.findUpCell(alliance))
            // Fix odometry drift, but only while holding still at a spot, never in the middle of a path.
            if (RobotConstants.USE_TAGS_TO_FIX_POSITION && !follower.isBusy()) {
                val velocity = follower.velocity()
                val speed = hypot(velocity.vx, velocity.vy)
                val fixed = relocalizer.correctedPosition(
                    camera.newSightings, pose, speed, Math.toDegrees(velocity.omega), System.nanoTime()
                )
                if (fixed != null) {
                    // Only x and y: the heading is left alone. The follower reads it at its next update.
                    pinpoint?.setPosX(fixed.x(), DistanceUnit.INCH)
                    pinpoint?.setPosY(fixed.y(), DistanceUnit.INCH)
                }
            }
        }
        val cameraCell = hive.cameraUpCell
        if (RobotConstants.USE_CAMERA_TO_FIND_UP_CELL && cameraCell != null) {
            hive.upCell = cameraCell
        }
        shooter.aimAt(pose, hive.target())

        // Teleop starts from wherever autonomous stopped.
        PoseStorage.save(pose, hive.upCell)

        telemetry.addData("Step", step)
        telemetry.addData("Time", "%.1f s".format(matchTimer.seconds()))
        telemetry.addData(
            "Position",
            "x %.1f  y %.1f  heading %.0f".format(pose.x(), pose.y(), Math.toDegrees(pose.heading()))
        )
        telemetry.addData("Target CELL", hive.upCell)
        telemetry.addData("Shooter", shooter.status())
        telemetry.update()
    }

    /** True once it is time to stop everything else and park. */
    private fun timeToPark(): Boolean {
        return matchTimer.seconds() > RobotConstants.AUTO_PARK_START_SECONDS
    }

    /** True when the robot is within 2 inches and 3 degrees of a pose. */
    private fun isCloseTo(target: Pose): Boolean {
        val pose = follower.pose()
        val headingError = Math.toDegrees(abs(Angle.normalizeSigned(target.heading() - pose.heading())))
        return Field.distance(pose, target) < 2.0 && headingError < 3.0
    }

    /** Start driving in a straight line from where we are to a pose on the field. */
    private fun startPath(target: Pose, slow: Boolean) {
        val here = follower.pose()

        if (Field.distance(here, target) < 2.0) {
            // Too short for a path. Just hold this pose (this still turns the robot if needed).
            follower.hold(target)
            return
        }

        // Turn to the final heading right away and keep it for the whole path.
        // (Do not use .linear() on Paths.line in Pedro 3.0.0: it turns the robot the wrong way,
        // so the robot ends the path facing its START heading.)
        var path = Paths.line(here, target).constant(target.heading())
        if (slow) {
            // Lower Pedro's top speed for this path only.
            path = path.with(Constants.foresightConfig.maxPathSpeed.at(RobotConstants.AUTO_SLOW_SPEED))
        }
        follower.follow(path)
    }

    /**
     * Drive to a pose (written for RED) and wait until we get there.
     * Skipped once it is time to park. Gives up on a path that takes too long (robot stuck).
     */
    private fun driveTo(redPose: Pose, slow: Boolean = false) {
        if (timeToPark()) return
        step = "driving"
        followPath(place(redPose), slow, stopWhenTimeToPark = true)
    }

    /** Follows a path to a pose on the field and waits until the robot gets there (or gives up). */
    private fun followPath(target: Pose, slow: Boolean, stopWhenTimeToPark: Boolean) {
        startPath(target, slow)
        val timer = ElapsedTime()
        while (opModeIsActive() && timer.seconds() < RobotConstants.AUTO_PATH_TIMEOUT_SECONDS) {
            if (stopWhenTimeToPark && timeToPark()) break
            updateRobot()
            // follower.isBusy() turns false when the path is done, but the robot may still be turning.
            if (!follower.isBusy() && isCloseTo(target)) break
        }
        if (follower.isBusy()) {
            // We gave up on this path (stuck, or out of time): stop pushing and stay where we are.
            follower.hold(follower.pose())
        }
    }

    /** Wait here (still aiming and holding position) for some seconds, or until it is time to park. */
    private fun waitSeconds(seconds: Double) {
        val timer = ElapsedTime()
        while (opModeIsActive() && timer.seconds() < seconds && !timeToPark()) {
            updateRobot()
        }
    }

    /**
     * Drive to the spot in front of the up CELL's opening (it is on the other side after the HIVE tips).
     * If that spot is on the other side of the HIVE, go around the HIVE along our alliance wall.
     */
    private fun goToShootingSpot() {
        shooter.isOn = true      // the flywheel speeds up while we drive
        val spot = if (hive.upCell == Field.startingUpCell(alliance)) {
            AutoPoses.SHOOT_AUDIENCE_SIDE     // in RED's view, the CELL that starts up opens toward the audience
        } else {
            AutoPoses.SHOOT_FAR_SIDE
        }
        val spotOnAudienceSide = spot.y() < Field.HIVE_Y
        if (onAudienceSide() && !spotOnAudienceSide) {
            driveTo(AutoPoses.LANE_AUDIENCE_SIDE)
            driveTo(AutoPoses.LANE_FAR_SIDE)
        }
        if (!onAudienceSide() && spotOnAudienceSide) {
            driveTo(AutoPoses.LANE_FAR_SIDE)
            driveTo(AutoPoses.LANE_AUDIENCE_SIDE)
        }
        driveTo(spot)
    }

    /** Shoot every POLLEN we are carrying. Returns false if we did not shoot (we still have them). */
    private fun shootAll(): Boolean {
        if (timeToPark()) return false
        step = "shooting"
        shooter.isOn = true

        // Wait (not too long) for the flywheel to get up to speed and the turret to aim.
        val waitTimer = ElapsedTime()
        while (opModeIsActive() && !shooter.isReady() && waitTimer.seconds() < RobotConstants.SPIN_UP_TIMEOUT_SECONDS && !timeToPark()) {
            updateRobot()
        }

        // Never shoot at the back of a CELL or past the turret's limits: those shots cannot go in.
        if (!shooter.onOpenSide || shooter.targetOutOfReach()) {
            step = "cannot shoot from here"
            shooter.isOn = false
            return false
        }
        // From our shooting spots the webcam sees our tags while the up CELL opens toward us. If the
        // webcam is in use and sees none of them here, the HIVE has probably tipped: keep the POLLEN for teleop.
        val cameraWorking = webcam?.isWorking() == true
        if (RobotConstants.USE_CAMERA_TO_FIND_UP_CELL && cameraWorking && timeSinceTagsSeen.seconds() > 0.5) {
            step = "no tags seen: the HIVE may have tipped, keeping our POLLEN"
            shooter.isOn = false
            return false
        }
        // If it never got ready (for example the flywheel is a bit slow), shoot anyway:
        // a few points are better than none.
        val shootAnyway = !shooter.isReady()

        // Feed POLLEN in. Feeding pauses while the flywheel speeds back up after each shot,
        // so we count only the time we really spent feeding.
        var secondsFed = 0.0
        val loopTimer = ElapsedTime()
        val giveUpTimer = ElapsedTime()
        while (opModeIsActive() && secondsFed < RobotConstants.FEED_SECONDS &&
            giveUpTimer.seconds() < RobotConstants.FEED_TIMEOUT_SECONDS && !timeToPark()
        ) {
            updateRobot()
            val feeding = shooter.isReady() || shootAnyway
            if (feeding) {
                intake.intake()
                transfer.feed()
                secondsFed += loopTimer.seconds()
            } else {
                intake.stop()
                transfer.stop()
            }
            loopTimer.reset()
        }
        intake.stop()
        transfer.stop()
        shooter.isOn = false     // save the battery until the next time we shoot
        return true
    }

    /** Creep into our GARDEN's POLLEN with the intake running. */
    private fun collectGarden() {
        if (timeToPark()) return
        intake.intake()
        transfer.whileIntaking()
        driveTo(AutoPoses.GARDEN_PICKUP, slow = true)
        step = "picking up GARDEN POLLEN"
        waitSeconds(RobotConstants.AUTO_GARDEN_INTAKE_SECONDS)
        intake.stop()
        transfer.stop()
    }

    /** Drive into our parking spot. This never gives up because of the clock: we always want to park. */
    private fun park() {
        step = "parking"
        shooter.isOn = false
        intake.stop()
        transfer.stop()

        val spot = place(if (parkInCorner) AutoPoses.PARK_CORNER else AutoPoses.PARK_MIDDLE)
        val lineUp = place(if (parkInCorner) AutoPoses.PARK_CORNER_LINE_UP else AutoPoses.PARK_MIDDLE_LINE_UP)
        // Keep the heading we have now until the line-up spot. When time runs out we may be right
        // next to a wall (for example in the GARDEN corner), where there is no room to turn.
        val heading = follower.pose().heading()
        if (onAudienceSide()) {
            val lane = place(AutoPoses.LANE_AUDIENCE_SIDE)
            followPath(lane.withHeading(heading), slow = false, stopWhenTimeToPark = false)
        }
        followPath(lineUp.withHeading(heading), slow = false, stopWhenTimeToPark = false)
        // There is room to turn at the line-up spot. Turn, then back into the LOADING ZONE.
        followPath(spot, slow = false, stopWhenTimeToPark = false)
    }

    /** Reads the INIT buttons on gamepad 1. Only changes numbers: nothing moves. */
    private fun chooseSettings() {
        if (gamepad1.dpadUpWasPressed()) {
            startDelaySeconds += 1.0
        }
        if (gamepad1.dpadDownWasPressed()) {
            startDelaySeconds -= 1.0
        }
        startDelaySeconds = startDelaySeconds.coerceIn(0.0, RobotConstants.AUTO_MAX_START_DELAY_SECONDS)

        // For both alliances, the corner half of the LOADING ZONE is on the drivers' left.
        if (gamepad1.dpadLeftWasPressed()) {
            parkInCorner = true
        }
        if (gamepad1.dpadRightWasPressed()) {
            parkInCorner = false
        }
    }

    private fun showInitTelemetry() {
        if (RobotConstants.USE_PANELS) {
            telemetry.addLine("Panels is ON. It is not allowed at competitions (rule R704.D): see USE_PANELS.")
        }
        telemetry.addData("Autonomous", "$alliance $routine")
        telemetry.addLine(placementHelp())
        telemetry.addLine("Load 4 POLLEN.")
        telemetry.addData("Start delay (dpad up/down)", "%.0f s".format(startDelaySeconds))
        telemetry.addData(
            "Park (dpad left/right)",
            if (parkInCorner) "LEFT half of the LOADING ZONE (corner)" else "RIGHT half of the LOADING ZONE (middle)"
        )
        val camera = webcam
        if (camera != null) {
            telemetry.addData("Webcam", camera.status())
            telemetry.addData("Webcam says the up CELL is", hive.cameraUpCell ?: "(none of our tags seen)")
        }
        telemetry.update()
    }

    /** Tells the drive team where to put the robot. */
    private fun placementHelp(): String {
        if (routine == Routine.GARDEN) {
            if (alliance == Alliance.RED) return "Back of the robot on the AUDIENCE wall, centered on tile C1, intake facing the HIVE."
            return "Back of the robot on the FAR wall, centered on tile D6, intake facing the HIVE."
        }
        if (alliance == Alliance.RED) return "Intake against the FAR wall on tile C6, right side 2 inches from the middle seam."
        return "Intake against the AUDIENCE wall on tile D1, right side 2 inches from the middle seam."
    }
}

// The name on the Driver Station says which wall the robot starts on.

@Autonomous(name = "Red Garden (audience wall)", group = "BioBuzz", preselectTeleOp = RobotConstants.TELEOP_RED)
class RedGardenAuto : BioBuzzAuto(Alliance.RED, Routine.GARDEN)

@Autonomous(name = "Red Go Around (far wall)", group = "BioBuzz", preselectTeleOp = RobotConstants.TELEOP_RED)
class RedGoAroundAuto : BioBuzzAuto(Alliance.RED, Routine.GO_AROUND)

@Autonomous(name = "Blue Garden (far wall)", group = "BioBuzz", preselectTeleOp = RobotConstants.TELEOP_BLUE)
class BlueGardenAuto : BioBuzzAuto(Alliance.BLUE, Routine.GARDEN)

@Autonomous(name = "Blue Go Around (audience wall)", group = "BioBuzz", preselectTeleOp = RobotConstants.TELEOP_BLUE)
class BlueGoAroundAuto : BioBuzzAuto(Alliance.BLUE, Routine.GO_AROUND)
