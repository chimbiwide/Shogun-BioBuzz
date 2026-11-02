package org.firstinspires.ftc.teamcode.auto

import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.follower.ManualDrive
import com.pedropathing.math.Pose
import com.pedropathing.revhub.drivetrains.Mecanum
import com.pedropathing.revhub.localizers.PinpointLocalizer
import com.pedropathing.utils.Angle
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.RobotConstants
import org.firstinspires.ftc.teamcode.game.Alliance
import org.firstinspires.ftc.teamcode.game.Field
import org.firstinspires.ftc.teamcode.game.HiveTracker
import org.firstinspires.ftc.teamcode.game.PoseStorage
import org.firstinspires.ftc.teamcode.pedro.Constants
import org.firstinspires.ftc.teamcode.tools.Intake
import org.firstinspires.ftc.teamcode.tools.Shooter
import org.firstinspires.ftc.teamcode.tools.Transfer
import kotlin.math.abs
import kotlin.math.hypot
import kotlin.math.min
import kotlin.math.sign

/*
 * A simple autonomous that does NOT need Pedro to be tuned. Use it until the Foresight Tuner is done.
 *
 * Start like the GARDEN routine: back of the robot on the wall our up CELL opens toward
 * (RED: AUDIENCE wall, tile C1. BLUE: FAR wall, tile D6), intake facing the HIVE. The robot shoots
 * its 4 pre-loaded POLLEN, then drives into the LOADING ZONE (LEAVE + PARK = 8 points).
 * At AUTO_PARK_START_SECONDS it stops waiting or shooting and drives to the LOADING ZONE.
 *
 * It drives with one simple rule: head straight for the next point, slowing down as it gets close.
 * The Pinpoint tells it where it is (the drive motors and Pinpoint must be set up, like for teleop).
 *
 * During INIT, gamepad 1: dpad up/down = start delay,
 * dpad left / right = park in the half of the LOADING ZONE on your LEFT (corner) / RIGHT (middle).
 */
abstract class SimpleAuto(private val alliance: Alliance) : LinearOpMode() {
    private lateinit var hubs: List<LynxModule>
    private lateinit var drivetrain: Mecanum
    private lateinit var localizer: PinpointLocalizer
    private lateinit var intake: Intake
    private lateinit var transfer: Transfer
    private lateinit var shooter: Shooter
    private lateinit var hive: HiveTracker
    // Never use "runtime" for match time: it starts counting at INIT, not at START.
    private val matchTimer = ElapsedTime()
    private var step = "waiting for start"

    // Settings the driver can change during INIT.
    private var startDelaySeconds = 0.0
    private var parkInCorner = false

    override fun runOpMode() {
        // Telemetry also goes to Panels, but only when Panels is turned on (see RobotConstants.USE_PANELS).
        if (RobotConstants.USE_PANELS) {
            telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry)
        }
        // Read all the motor encoders at once, one time per loop (updateRobot() clears the old readings).
        hubs = hardwareMap.getAll(LynxModule::class.java)
        for (hub in hubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }

        drivetrain = Mecanum(hardwareMap, Constants.drivetrainConfig)
        localizer = PinpointLocalizer(hardwareMap, Constants.localizerConfig)
        intake = Intake(hardwareMap)
        transfer = Transfer(hardwareMap)
        shooter = Shooter(hardwareMap)
        hive = HiveTracker(alliance)

        val startPose = Field.forAlliance(AutoPoses.AUDIENCE_START, alliance)
        localizer.setPose(startPose)
        // Point the turret and hood, and make sure the transfer is stopped, once, now.
        // Nothing may move after INIT is done (rule G304.H).
        shooter.aimAt(startPose, hive.target())
        transfer.stop()

        while (opModeInInit()) {
            chooseSettings()
            if (RobotConstants.USE_PANELS) {
                telemetry.addLine("Panels is ON. It is not allowed at competitions (rule R704.D): see USE_PANELS.")
            }
            telemetry.addData("Autonomous", "$alliance SIMPLE (no Pedro)")
            if (alliance == Alliance.RED) {
                telemetry.addLine("Back of the robot on the AUDIENCE wall, centered on tile C1, intake facing the HIVE.")
            } else {
                telemetry.addLine("Back of the robot on the FAR wall, centered on tile D6, intake facing the HIVE.")
            }
            telemetry.addLine("Load 4 POLLEN.")
            telemetry.addData("Start delay (dpad up/down)", "%.0f s".format(startDelaySeconds))
            telemetry.addData(
                "Park (dpad left/right)",
                if (parkInCorner) "LEFT half of the LOADING ZONE (corner)" else "RIGHT half of the LOADING ZONE (middle)"
            )
            telemetry.update()
            sleep(20)   // OK here: before START there is nothing to keep updating
        }
        if (isStopRequested) return

        matchTimer.reset()
        // Set the start position again: the Pinpoint can miss a position set while it is still calibrating.
        localizer.setPose(startPose)

        shooter.isOn = true                  // start the flywheel now, so it is up to speed sooner
        waitSeconds(startDelaySeconds)
        shootAll()                           // the 4 POLLEN we started with

        driveTo(AutoPoses.LEAVE_AUDIENCE_WALL)
        driveTo(AutoPoses.LANE_AUDIENCE_SIDE)
        if (parkInCorner) {
            driveTo(AutoPoses.PARK_CORNER_LINE_UP)
            driveTo(AutoPoses.PARK_CORNER)
        } else {
            driveTo(AutoPoses.PARK_MIDDLE_LINE_UP)
            driveTo(AutoPoses.PARK_MIDDLE)
        }

        step = "parked"
        while (opModeIsActive()) {
            updateRobot()
        }
    }

    /** Everything that must happen every loop. Every waiting loop below calls this. */
    private fun updateRobot() {
        // Throw away last loop's motor readings (MANUAL bulk caching, see runOpMode).
        for (hub in hubs) {
            hub.clearBulkCache()
        }
        localizer.update()
        val pose = localizer.pose()
        shooter.aimAt(pose, hive.target())
        // Teleop starts from wherever autonomous stopped.
        PoseStorage.save(pose, hive.upCell)

        telemetry.addData("Step", step)
        telemetry.addData("Time", "%.1f s".format(matchTimer.seconds()))
        telemetry.addData(
            "Position",
            "x %.1f  y %.1f  heading %.0f".format(pose.x(), pose.y(), Math.toDegrees(pose.heading()))
        )
        telemetry.addData("Shooter", shooter.status())
        telemetry.update()
    }

    /**
     * Drive to a pose (written for RED): head straight for it, slowing down as we get close.
     * Gives up after AUTO_PATH_TIMEOUT_SECONDS (for example if something is in the way).
     */
    private fun driveTo(redPose: Pose) {
        step = "driving"
        val target = Field.forAlliance(redPose, alliance)
        val closeEnoughInches = 2.0
        val closeEnoughRadians = Math.toRadians(5.0)
        val timer = ElapsedTime()
        while (opModeIsActive() && timer.seconds() < RobotConstants.AUTO_PATH_TIMEOUT_SECONDS) {
            updateRobot()
            val pose = localizer.pose()
            val xToGo = target.x() - pose.x()
            val yToGo = target.y() - pose.y()
            val distanceToGo = hypot(xToGo, yToGo)
            val turnToGo = Angle.normalizeSigned(target.heading() - pose.heading())   // radians, -PI to PI
            if (distanceToGo < closeEnoughInches && abs(turnToGo) < closeEnoughRadians) {
                break   // we are there
            }

            // More power when far away, but never more than SIMPLE_AUTO_MAX_POWER.
            // Until we are close enough, add a little extra so the wheels keep turning.
            var power = distanceToGo * RobotConstants.SIMPLE_AUTO_GAIN
            if (distanceToGo > closeEnoughInches) {
                power += RobotConstants.SIMPLE_AUTO_MIN_POWER
            }
            power = min(power, RobotConstants.SIMPLE_AUTO_MAX_POWER)
            var xPower = 0.0
            var yPower = 0.0
            if (distanceToGo > 0.1) {
                // Split the power between x and y in the same ratio as the distance left,
                // so the robot drives in a straight line.
                xPower = xToGo / distanceToGo * power
                yPower = yToGo / distanceToGo * power
            }

            // The same for turning. sign() is +1 for a left (counterclockwise) turn, -1 for a right turn.
            var turnPower = turnToGo * RobotConstants.SIMPLE_AUTO_TURN_GAIN
            if (abs(turnToGo) > closeEnoughRadians) {
                turnPower += sign(turnToGo) * RobotConstants.SIMPLE_AUTO_MIN_TURN_POWER
            }
            turnPower = turnPower.coerceIn(-RobotConstants.SIMPLE_AUTO_MAX_POWER, RobotConstants.SIMPLE_AUTO_MAX_POWER)

            // Field-centric with offset 0: the first number drives toward +x on the field, the second toward +y.
            drivetrain.drive(ManualDrive.fieldCentric(xPower, yPower, turnPower, pose.heading(), 0.0), true)
        }
        drivetrain.stop()
    }

    /** True once it is time to stop waiting or shooting and go park. */
    private fun timeToPark(): Boolean {
        return matchTimer.seconds() > RobotConstants.AUTO_PARK_START_SECONDS
    }

    /** Wait here (still aiming) for some seconds, or until it is time to park. */
    private fun waitSeconds(seconds: Double) {
        val timer = ElapsedTime()
        while (opModeIsActive() && timer.seconds() < seconds && !timeToPark()) {
            updateRobot()
        }
    }

    /** Shoot every POLLEN we are carrying (like shootAll() in BioBuzzAuto, without the webcam's "no tags seen" check). */
    private fun shootAll() {
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
            return
        }
        // If it never got ready, shoot anyway: a few points are better than none.
        val shootAnyway = !shooter.isReady()

        // Feed POLLEN in, counting only the time we really spent feeding.
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
        shooter.isOn = false
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
}

@Autonomous(name = "Red Simple (audience wall, no Pedro)", group = "BioBuzz", preselectTeleOp = RobotConstants.TELEOP_RED)
class RedSimpleAuto : SimpleAuto(Alliance.RED)

@Autonomous(name = "Blue Simple (far wall, no Pedro)", group = "BioBuzz", preselectTeleOp = RobotConstants.TELEOP_BLUE)
class BlueSimpleAuto : SimpleAuto(Alliance.BLUE)
