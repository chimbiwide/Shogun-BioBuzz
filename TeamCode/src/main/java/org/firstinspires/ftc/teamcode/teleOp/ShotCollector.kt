package org.firstinspires.ftc.teamcode.teleOp

import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.follower.ManualDrive
import com.pedropathing.math.Pose
import com.pedropathing.math.Velocity
import com.pedropathing.revhub.drivetrains.Mecanum
import com.pedropathing.revhub.localizers.PinpointLocalizer
import com.pedropathing.utils.Angle
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import org.firstinspires.ftc.teamcode.RobotConstants
import org.firstinspires.ftc.teamcode.game.Alliance
import org.firstinspires.ftc.teamcode.game.Field
import org.firstinspires.ftc.teamcode.game.HiveCell
import org.firstinspires.ftc.teamcode.game.Relocalizer
import org.firstinspires.ftc.teamcode.game.ShotFeatures
import org.firstinspires.ftc.teamcode.logging.ShotLogger
import org.firstinspires.ftc.teamcode.pedro.Constants
import org.firstinspires.ftc.teamcode.tools.Battery
import org.firstinspires.ftc.teamcode.tools.Intake
import org.firstinspires.ftc.teamcode.tools.Shooter
import org.firstinspires.ftc.teamcode.tools.ShotLogData
import org.firstinspires.ftc.teamcode.tools.Transfer
import org.firstinspires.ftc.teamcode.tools.Webcam
import kotlin.math.hypot

/*
 * Shot Collector: collects shot data to train the shot model. Uses the RED half of the field.
 * BIOBUZZ_GUIDE.md, section 10, explains how to run a data collection session.
 *
 * Load ONE POLLEN at a time. Shoot it, watch where it goes, press its label, then load the next one.
 * A spotter stands at the wall the CELL's opening faces and tells the operator what happened.
 * While the robot moves it aims a little against its motion ("lead", BALL_FLIGHT_SECONDS), because the
 * ball keeps the robot's speed.
 * Gamepad 2 buzzes: one long buzz = READY to shoot. One short blip = a shot was found: label it.
 * Two short blips = the robot saw NO shot from that feed, so no row was made: do not label it, shoot again.
 *
 * DRIVER (gamepad 1)
 *   left stick ........ drive (field-centric, like teleop)       right stick X ... turn
 *   right bumper ...... hold for slow mode                       right trigger ... intake
 *   left trigger ...... un-jam                                   cross ........... hold to face the target CELL
 *   triangle .......... switch the target CELL (press it when the HIVE tips)
 *   OPTIONS + dpad down  robot is in the RED corner, back on the red wall, left side on the far wall: reset there
 *
 * OPERATOR (gamepad 2)
 *   triangle .......... flywheel on / off
 *   right trigger ..... shoot ONE ball (when READY, and only after the last shot has its label)
 *   left trigger ...... un-jam                                   bumpers ......... nudge the aim left / right
 *   square ............ SWEEP mode (a random speed and hood change after every shot) <-> TEST mode (no changes)
 *   Labels for the last shot (press another one before feeding the next ball to fix a mistake):
 *     cross ........... IN (went in)
 *     dpad down ....... SHORT (too low or too weak)          dpad up ......... LONG (too high or too strong)
 *     dpad left ....... WIDE LEFT                            dpad right ...... WIDE RIGHT
 *                       (left and right as seen from behind the robot, looking at the CELL)
 *     circle .......... OTHER (bounced out, hit the frame, nobody saw)
 *     SHARE ........... NOT A SHOT (no ball came out)
 */
@TeleOp(name = "Shot Collector", group = "BioBuzz Tuning")
class ShotCollector : OpMode() {
    private lateinit var hubs: List<LynxModule>
    private lateinit var drivetrain: Mecanum
    private lateinit var localizer: PinpointLocalizer
    private lateinit var intake: Intake
    private lateinit var transfer: Transfer
    private lateinit var shooter: Shooter
    private lateinit var battery: Battery
    private lateinit var logger: ShotLogger
    private var webcam: Webcam? = null
    private var pinpoint: GoBildaPinpointDriver? = null
    private val relocalizer = Relocalizer()
    private val clock = ElapsedTime()        // seconds since START
    private val loopTimer = ElapsedTime()
    private val feedTimer = ElapsedTime()

    private var targetCell = HiveCell.AUDIENCE_SIDE   // RED's CELL that starts up
    private var sweepMode = true
    private var feedingOneBall = false
    private var wasReady = false
    private var feedsWithoutShot = 0
    private var lastLabelText = "no labels yet"

    override fun init() {
        // Telemetry also goes to Panels, but only when Panels is turned on (see RobotConstants.USE_PANELS).
        if (RobotConstants.USE_PANELS) {
            telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry)
        }
        // Read all the motor encoders at once, one time per loop (cleared at the start of loop()).
        hubs = hardwareMap.getAll(LynxModule::class.java)
        for (hub in hubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }
        drivetrain = Mecanum(hardwareMap, Constants.drivetrainConfig)
        localizer = PinpointLocalizer(hardwareMap, Constants.localizerConfig)
        pinpoint = hardwareMap.getAll(GoBildaPinpointDriver::class.java).firstOrNull()
        intake = Intake(hardwareMap)
        transfer = Transfer(hardwareMap)
        shooter = Shooter(hardwareMap)
        battery = Battery(hardwareMap)
        if (RobotConstants.USE_WEBCAM) {
            webcam = Webcam(hardwareMap)
        }
    }

    override fun init_loop() {
        webcam?.applyCameraSettings()   // short exposure, once the camera is streaming
        telemetry.addLine("Put the robot in the RED corner: back on the red wall, left side on the far wall.")
        telemetry.addLine("Load ONE POLLEN at a time. Spotter at the wall the CELL opening faces.")
        telemetry.addData("Webcam", webcam?.status() ?: "not used (USE_WEBCAM = false)")
        telemetry.update()
    }

    override fun start() {
        clock.reset()
        loopTimer.reset()
        localizer.setPose(Field.cornerPose(Alliance.RED))
        relocalizer.forgetOldPictures(System.nanoTime())
        logger = ShotLogger(AppUtil.ROBOT_DATA_DIR, "Shot Collector", Double.POSITIVE_INFINITY)
        pickNextOffsets()
    }

    override fun loop() {
        for (hub in hubs) {
            hub.clearBulkCache()
        }
        val loopMs = loopTimer.milliseconds()
        loopTimer.reset()
        localizer.update()
        var pose = localizer.pose()
        val velocity = localizer.velocity()
        battery.update()

        // The webcam fixes odometry drift (if turned on), so the logged distances stay right.
        val camera = webcam
        if (camera != null) {
            camera.applyCameraSettings()   // does nothing once the exposure is set
            camera.update()
            if (RobotConstants.USE_TAGS_TO_FIX_POSITION) {
                val fixed = relocalizer.correctedPosition(
                    camera.newSightings, pose, hypot(velocity.vx, velocity.vy),
                    Math.toDegrees(velocity.omega), System.nanoTime()
                )
                if (fixed != null) {
                    pinpoint?.setPosX(fixed.x(), DistanceUnit.INCH)
                    pinpoint?.setPosY(fixed.y(), DistanceUnit.INCH)
                    pose = fixed
                }
            }
        }

        drive(pose)
        operatorButtons()

        // Aim a little against our motion (lead), because the ball keeps the robot's speed.
        val target = Field.openingPosition(Alliance.RED, targetCell)
        val aimPoint = ShotFeatures.leadPoint(target, pose, velocity, RobotConstants.BALL_FLIGHT_SECONDS)
        shooter.aimAt(pose, aimPoint, target)

        val feeding = intakeAndShoot()

        val shotFound = logger.update(
            ShotLogData.make(
                clock.seconds(), loopMs, Alliance.RED, targetCell, if (sweepMode) "SWEEP" else "TEST", pose, velocity,
                target, aimPoint, RobotConstants.BALL_FLIGHT_SECONDS, shooter, false, feeding, false,
                battery.volts, relocalizer.secondsSinceFix(System.nanoTime())
            )
        )
        if (shotFound) {
            feedingOneBall = false     // that ball is gone: stop feeding
            gamepad2.rumbleBlips(1)    // one short blip: "label it now"
        }

        // One long buzz when the shooter becomes ready.
        val ready = shooter.isReady()
        if (ready && !wasReady) {
            gamepad2.rumble(300)
        }
        wasReady = ready

        showTelemetry(pose, velocity, target, aimPoint, loopMs)
    }

    override fun stop() {
        shooter.isOn = false
        shooter.flywheel.setRpm(0.0)
        intake.stop()
        transfer.stop()
        drivetrain.stop()
        if (::logger.isInitialized) {
            logger.close(clock.seconds())
        }
    }

    private fun drive(pose: Pose) {
        val forward = -gamepad1.left_stick_y.toDouble()
        val strafe = -gamepad1.left_stick_x.toDouble()
        var turn = -gamepad1.right_stick_x.toDouble()
        if (gamepad1.cross) {
            // Turn so the robot faces the target CELL.
            val wantedHeading = Field.headingTo(pose, Field.openingPosition(Alliance.RED, targetCell))
            turn = (Angle.normalizeSigned(wantedHeading - pose.heading()) * RobotConstants.AIM_TURN_GAIN).coerceIn(-1.0, 1.0)
        }
        var speed = 1.0
        if (gamepad1.right_bumper) {
            speed = RobotConstants.SLOW_MODE_SPEED
        }
        // Field-centric for the RED drivers (they look toward +x).
        drivetrain.drive(ManualDrive.fieldCentric(forward * speed, strafe * speed, turn * speed, pose.heading(), 0.0), true)

        if (gamepad1.triangleWasPressed()) {
            targetCell = targetCell.other()
        }
        // Read the dpad first: reading it uses up the press.
        if (gamepad1.dpadDownWasPressed() && gamepad1.options) {
            localizer.setPose(Field.cornerPose(Alliance.RED))
            relocalizer.forgetOldPictures(System.nanoTime())
            gamepad1.rumble(300)
        }
    }

    private fun operatorButtons() {
        if (gamepad2.triangleWasPressed()) {
            shooter.isOn = !shooter.isOn
        }
        if (gamepad2.squareWasPressed()) {
            sweepMode = !sweepMode
            pickNextOffsets()
        }
        // Nudge the aim (positive = more to the left).
        if (gamepad2.leftBumperWasPressed()) {
            shooter.turretTrimDegrees += 1.0
        }
        if (gamepad2.rightBumperWasPressed()) {
            shooter.turretTrimDegrees -= 1.0
        }

        // Labels. Each button is read once per loop.
        var label: String? = null
        if (gamepad2.crossWasPressed()) label = ShotLogger.IN
        if (gamepad2.dpadDownWasPressed()) label = ShotLogger.SHORT
        if (gamepad2.dpadUpWasPressed()) label = ShotLogger.LONG
        if (gamepad2.dpadLeftWasPressed()) label = ShotLogger.WIDE_LEFT
        if (gamepad2.dpadRightWasPressed()) label = ShotLogger.WIDE_RIGHT
        if (gamepad2.circleWasPressed()) label = ShotLogger.OTHER
        if (gamepad2.shareWasPressed()) label = ShotLogger.NOT_A_SHOT
        if (label != null) {
            if (logger.waitingForLabel > 0) {
                logger.label(label, clock.seconds())
                lastLabelText = "shot #%d = %s (press another label to fix it, before the next ball)".format(logger.shotsFound, label)
                pickNextOffsets()   // a new random setting for the next ball
            } else if (logger.relabelLastShot(label, clock.seconds())) {
                lastLabelText = "shot #%d changed to %s".format(logger.shotsFound, label)
            }
            // Otherwise there is nothing to label (for example the robot saw no shot): the press is ignored.
        }
    }

    /** One ball per press. Returns true while feeding a ball into the flywheel. */
    private fun intakeAndShoot(): Boolean {
        val justPressed = gamepad2.rightTriggerWasPressed()   // true once per press, like the buttons
        if (justPressed && !feedingOneBall && logger.waitingForLabel == 0 && shooter.isReady()) {
            feedingOneBall = true
            feedTimer.reset()
        }
        if (feedingOneBall && feedTimer.seconds() > RobotConstants.COLLECTOR_FEED_TIMEOUT_SECONDS) {
            // No shot was found: maybe no ball was loaded, or the detector missed it. No row was made.
            feedingOneBall = false
            feedsWithoutShot += 1
            lastLabelText = "NO SHOT SEEN on the last feed: do not label it, shoot again"
            gamepad2.rumbleBlips(2)
        }

        if (gamepad1.left_trigger_pressed || gamepad2.left_trigger_pressed) {
            feedingOneBall = false
            intake.eject()
            transfer.reverse()
        } else if (feedingOneBall) {
            intake.intake()
            transfer.feed()
            return true
        } else if (gamepad1.right_trigger_pressed) {
            intake.intake()
            transfer.whileIntaking()
        } else {
            intake.stop()
            transfer.stop()
        }
        return false
    }

    /** SWEEP mode: a random change to the shot table's speed and hood for the next ball. TEST mode: none. */
    private fun pickNextOffsets() {
        if (sweepMode) {
            shooter.rpmOffset = RobotConstants.SWEEP_RPM_OFFSETS.random()
            shooter.hoodOffset = RobotConstants.SWEEP_HOOD_OFFSETS.random()
        } else {
            shooter.rpmOffset = 0.0
            shooter.hoodOffset = 0.0
        }
    }

    private fun showTelemetry(pose: Pose, velocity: Velocity, target: Pose, aimPoint: Pose, loopMs: Double) {
        telemetry.addData("Shooter", shooter.status())
        if (logger.waitingForLabel > 0) {
            telemetry.addLine("LABEL SHOT #%d: cross IN | dpad SHORT/LONG/WIDE | circle OTHER | SHARE no shot".format(logger.shotsFound))
        } else {
            telemetry.addData("Last label", lastLabelText)
        }
        telemetry.addData(
            "Mode",
            "%s: next ball %+.0f rpm, hood %+.2f   (gamepad 2 square switches)".format(
                if (sweepMode) "SWEEP" else "TEST", shooter.rpmOffset, shooter.hoodOffset
            )
        )
        val features = ShotFeatures.compute(pose, velocity, target, targetCell)
        val turret = ShotFeatures.turretPosition(pose)
        val leadDegrees = Math.toDegrees(Angle.normalizeSigned(Field.headingTo(turret, aimPoint) - Field.headingTo(turret, target)))
        telemetry.addData(
            "Now",
            "%.0f in | toward %+.0f | left %+.0f in/s | turn %.0f deg/s | lead %.0f deg".format(
                features.distanceInches, features.towardSpeed, features.leftSpeed, features.turnDegreesPerSecond, leadDegrees
            )
        )
        telemetry.addData(
            "Flywheel",
            "%.0f / %.0f rpm (table %.0f %+.0f) | hood %.2f (table %.2f %+.2f)".format(
                shooter.flywheel.currentRpm(), shooter.flywheel.targetRpm, shooter.setting.flywheelRpm,
                shooter.rpmOffset, shooter.hood.position, shooter.setting.hoodPosition, shooter.hoodOffset
            )
        )
        telemetry.addData("Target CELL", "RED $targetCell (gamepad 1 triangle switches)")
        telemetry.addData("Last shot", logger.lastShotSummary())
        telemetry.addData(
            "Log",
            "%s | %d rows | feeds without a shot: %d".format(logger.fileName(), logger.rowsWritten, feedsWithoutShot)
        )
        telemetry.addData(
            "Position",
            "x %.1f  y %.1f  heading %.0f | battery %.1f V | loop %.0f ms".format(
                pose.x(), pose.y(), Math.toDegrees(pose.heading()), battery.volts, loopMs
            )
        )
        telemetry.update()
    }
}
