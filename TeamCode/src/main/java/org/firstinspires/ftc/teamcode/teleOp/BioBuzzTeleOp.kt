package org.firstinspires.ftc.teamcode.teleOp

import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.follower.ManualDrive
import com.pedropathing.math.Pose
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
import org.firstinspires.ftc.teamcode.game.HiveTracker
import org.firstinspires.ftc.teamcode.game.PoseStorage
import org.firstinspires.ftc.teamcode.game.Relocalizer
import org.firstinspires.ftc.teamcode.game.ShotFeatures
import org.firstinspires.ftc.teamcode.logging.ShotLogger
import org.firstinspires.ftc.teamcode.pedro.Constants
import org.firstinspires.ftc.teamcode.tools.Battery
import org.firstinspires.ftc.teamcode.tools.Intake
import org.firstinspires.ftc.teamcode.tools.LimeLight
import org.firstinspires.ftc.teamcode.tools.Shooter
import org.firstinspires.ftc.teamcode.tools.ShotLogData
import org.firstinspires.ftc.teamcode.tools.Transfer
import org.firstinspires.ftc.teamcode.tools.Webcam
import kotlin.math.PI
import kotlin.math.hypot

/*
 * BIOBUZZ driver-controlled period. Pick "TeleOp Red" or "TeleOp Blue" (bottom of this file).
 *
 * DRIVER (gamepad 1)
 *   left stick ........ drive (field-centric: push away from you = robot goes away from you)
 *   right stick X ..... turn
 *   right bumper ...... hold for slow mode
 *   right trigger ..... intake
 *   left trigger ...... un-jam (intake + transfer backwards; also drops an extra POLLEN or a NECTAR)
 *   cross ............. hold to turn and face the target CELL (the turret can only turn a little)
 *   left bumper ....... hold to turn toward the closest POLLEN (Limelight)
 *   OPTIONS + dpad down  robot is in the corner on your LEFT along your alliance wall: reset position there
 *   OPTIONS + SHARE ... robot is facing straight away from you: reset heading only
 *
 * OPERATOR (gamepad 2)
 *   triangle .......... flywheel on / off
 *   right trigger ..... hold to shoot (POLLEN only feed in while the shooter is READY)
 *   cross + right trigger  shoot before the flywheel is exactly at speed (never from the WRONG SIDE, except in manual mode)
 *   left trigger ...... un-jam (intake + transfer backwards)
 *   dpad left/right ... the up CELL is the one on your left / right (press this every time the HIVE tips)
 *   dpad up/down ...... shots go farther / shorter
 *   bumpers ........... nudge the aim left / right
 *   square ............ manual mode on/off (turret straight ahead; you point the robot; NO wrong-side check)
 *   circle / SHARE .... only when LOG_SHOTS_IN_TELEOP is on (practice): the shot just found (one short blip) went
 *                       IN / MISSED. Shoot one ball at a time: with two or more waiting, they are saved UNLABELED.
 *
 * Press START only when TELEOP begins: the turret and hood move as soon as you press it (rule G403).
 * At 0:00 the flywheel stops by itself; then press STOP on the Driver Station (rule G404).
 *
 * We only pick up POLLEN. Do not pick up NECTAR, and never hold the other alliance's NECTAR (rule G408).
 * Never hold more than 4 game pieces (rule G407): shoot before you pick up more.
 */
abstract class BioBuzzTeleOp(private val alliance: Alliance) : OpMode() {
    private lateinit var hubs: List<LynxModule>
    private lateinit var drivetrain: Mecanum
    private lateinit var localizer: PinpointLocalizer
    private lateinit var intake: Intake
    private lateinit var transfer: Transfer
    private lateinit var shooter: Shooter
    private lateinit var limelight: LimeLight
    private lateinit var hive: HiveTracker
    private lateinit var battery: Battery
    private var webcam: Webcam? = null
    private var pinpoint: GoBildaPinpointDriver? = null
    private var shotLogger: ShotLogger? = null
    private val relocalizer = Relocalizer()
    // Never use "runtime" for match time: it starts counting at INIT, not at START.
    private val matchTimer = ElapsedTime()
    private val loopTimer = ElapsedTime()

    private var startPose = Field.cornerPose(alliance)
    private var positionCarriedOver = false
    private var manualMode = false
    private var wasReady = false
    private var parkWarningGiven = false
    private var flywheelStoppedAtEnd = false
    private var feeding = false     // pushing POLLEN into the flywheel this loop
    private var forced = false      // ... with the operator holding cross

    // RED drivers look toward +x, BLUE drivers look the other way (180 degrees).
    private val driverHeading = if (alliance == Alliance.RED) 0.0 else PI

    // RED drivers have the far-side CELL on their left, BLUE drivers have the audience-side CELL on their left.
    private val cellOnLeft = if (alliance == Alliance.RED) HiveCell.FAR_SIDE else HiveCell.AUDIENCE_SIDE

    override fun init() {
        // Telemetry also goes to Panels, but only when Panels is turned on (see RobotConstants.USE_PANELS).
        if (RobotConstants.USE_PANELS) {
            telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry)
        }
        // Read all the motor encoders at once, one time per loop, so the loop runs faster.
        // MANUAL means we throw away the old readings ourselves, at the start of loop().
        hubs = hardwareMap.getAll(LynxModule::class.java)
        for (hub in hubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }

        // We use the Pedro drivetrain and Pinpoint directly, so teleop works even before Pedro is tuned.
        drivetrain = Mecanum(hardwareMap, Constants.drivetrainConfig)
        localizer = PinpointLocalizer(hardwareMap, Constants.localizerConfig)
        // The same Pinpoint, to correct only x and y when the webcam fixes our position.
        pinpoint = hardwareMap.getAll(GoBildaPinpointDriver::class.java).firstOrNull()
        intake = Intake(hardwareMap)
        transfer = Transfer(hardwareMap)
        shooter = Shooter(hardwareMap)
        limelight = LimeLight(hardwareMap)
        hive = HiveTracker(alliance)
        battery = Battery(hardwareMap)
        if (RobotConstants.USE_WEBCAM) {
            webcam = Webcam(hardwareMap)
        }

        // If autonomous (or this teleop, before a restart) just ran, carry on from where it stopped.
        val savedPose = PoseStorage.pose
        val savedCell = PoseStorage.upCell
        if (PoseStorage.isFresh() && savedPose != null) {
            startPose = savedPose
            positionCarriedOver = true
            if (savedCell != null) {
                hive.upCell = savedCell
            }
        }
        // The position is set in start(): the Pinpoint is still calibrating right now.
        // Nothing moves during INIT (rule G403), so we do not touch the servos or motors here.
    }

    override fun init_loop() {
        webcam?.applyCameraSettings()   // short exposure, once the camera is streaming

        if (RobotConstants.USE_PANELS) {
            telemetry.addLine("Panels is ON. It is not allowed at competitions (rule R704.D): see USE_PANELS.")
        }
        telemetry.addData("Alliance", alliance)
        if (positionCarriedOver) {
            telemetry.addLine("Start position: where the last OpMode left the robot (autonomous,")
            telemetry.addLine("or this teleop before a restart). If the robot was moved since then, put it in")
            telemetry.addLine("the corner on your LEFT and press OPTIONS + dpad down right after START.")
        } else {
            telemetry.addLine("Start position: put the robot in the corner on your LEFT,")
            telemetry.addLine("back on your alliance wall, before pressing START.")
        }
        telemetry.addData("Will aim at", "${cellName(hive.upCell)} CELL (if the HIVE tipped, press dpad after START)")
        telemetry.addLine("Press START only when TELEOP begins: the turret moves right away (rule G403).")
        if (limelight.isConnected()) {
            telemetry.addData("Limelight", "answering, pipeline %s".format(limelight.pipelineIndex()))
        } else {
            telemetry.addData("Limelight", "NOT ANSWERING (missing, unplugged or still starting up)")
        }
        val camera = webcam
        if (camera != null) {
            telemetry.addData("Webcam", camera.status())
        }
        if (RobotConstants.LOG_SHOTS_IN_TELEOP) {
            telemetry.addLine("Shot logging is ON (practice only): label shots with gamepad 2 circle / SHARE.")
        }
        telemetry.update()
    }

    override fun start() {
        matchTimer.reset()
        loopTimer.reset()
        localizer.setPose(startPose)
        relocalizer.forgetOldPictures(System.nanoTime())
        if (RobotConstants.LOG_SHOTS_IN_TELEOP) {
            val name = if (alliance == Alliance.RED) RobotConstants.TELEOP_RED else RobotConstants.TELEOP_BLUE
            shotLogger = ShotLogger(AppUtil.ROBOT_DATA_DIR, name, RobotConstants.SHOT_LABEL_TIMEOUT_SECONDS)
        }
    }

    override fun loop() {
        // Throw away last loop's motor readings (MANUAL bulk caching, see init()).
        for (hub in hubs) {
            hub.clearBulkCache()
        }
        val loopMs = loopTimer.milliseconds()
        loopTimer.reset()
        localizer.update()
        var pose = localizer.pose()
        val velocity = localizer.velocity()   // on the field: vx, vy in inches per second, omega in radians per second
        battery.update()

        // The webcam: fix odometry drift (if turned on), and a hint about which CELL is up.
        val camera = webcam
        if (camera != null) {
            camera.applyCameraSettings()   // does nothing once the exposure is set
            camera.update()
            if (RobotConstants.USE_TAGS_TO_FIX_POSITION) {
                val speed = hypot(velocity.vx, velocity.vy)
                val turnRate = Math.toDegrees(velocity.omega)
                val fixed = relocalizer.correctedPosition(camera.newSightings, pose, speed, turnRate, System.nanoTime())
                if (fixed != null) {
                    moveOdometryTo(fixed)
                    pose = fixed
                }
            }
            // Only a hint on the screen. The operator decides which CELL is up.
            hive.updateFromCamera(camera.findUpCell(alliance))
        }

        drive(pose)
        operatorButtons()

        // TELEOP is over: stop the flywheel once (rule G404). Then press STOP on the Driver Station.
        // (In a long practice run the operator can turn it back on.)
        if (!flywheelStoppedAtEnd && matchTimer.seconds() > RobotConstants.TELEOP_SECONDS) {
            shooter.isOn = false
            flywheelStoppedAtEnd = true
        }

        // Aim at the up CELL's opening. When shooting on the move is turned on, aim at a "lead" point.
        val target = hive.target()
        var aimPoint = target
        var flightSeconds = 0.0
        if (RobotConstants.USE_LEAD_IN_TELEOP) {
            flightSeconds = RobotConstants.BALL_FLIGHT_SECONDS
            aimPoint = ShotFeatures.leadPoint(target, pose, velocity, flightSeconds)
        }
        if (manualMode) {
            shooter.aimStraight(RobotConstants.MANUAL_DISTANCE_INCHES)
        } else {
            shooter.aimAt(pose, aimPoint, target)
        }
        intakeAndShoot()

        // Practice data for the shot model (only when LOG_SHOTS_IN_TELEOP is on).
        val shotFound = shotLogger?.update(
            ShotLogData.make(
                matchTimer.seconds(), loopMs, alliance, hive.upCell, "TELEOP", pose, velocity, target, aimPoint,
                flightSeconds, shooter, manualMode, feeding, forced, battery.volts,
                relocalizer.secondsSinceFix(System.nanoTime())
            )
        ) == true
        if (shotFound) {
            gamepad2.rumbleBlips(1)   // one short blip: a shot was found, label it
        }

        // Buzz the operator's controller when the shooter becomes ready.
        val ready = shooter.isReady()
        if (ready && !wasReady) {
            gamepad2.rumble(200)
        }
        wasReady = ready

        // 20 seconds left: buzz the driver, time to go park.
        if (!parkWarningGiven && matchTimer.seconds() > RobotConstants.PARK_WARNING_SECONDS) {
            gamepad1.rumbleBlips(3)
            parkWarningGiven = true
        }

        // If this teleop has to be restarted, it carries on from here.
        PoseStorage.save(pose, hive.upCell)

        showTelemetry(pose)
    }

    override fun stop() {
        shooter.isOn = false
        shooter.flywheel.setRpm(0.0)
        intake.stop()
        transfer.stop()
        drivetrain.stop()
        limelight.stop()
        shotLogger?.close(matchTimer.seconds())
    }

    private fun drive(pose: Pose) {
        val forward = -gamepad1.left_stick_y.toDouble()
        val strafe = -gamepad1.left_stick_x.toDouble()
        var turn = -gamepad1.right_stick_x.toDouble()

        if (gamepad1.cross) {
            // Turn so the robot faces the target CELL. Positive turn = counterclockwise (left).
            val wantedHeading = Field.headingTo(pose, hive.target())
            val headingError = Angle.normalizeSigned(wantedHeading - pose.heading())
            turn = (headingError * RobotConstants.AIM_TURN_GAIN).coerceIn(-1.0, 1.0)
        } else if (gamepad1.left_bumper) {
            // The Limelight turns the robot toward the closest POLLEN. You still drive with the left stick.
            val ball = limelight.closestPollen(pose)
            if (ball != null) {
                val bearing = Math.toRadians(ball.bearingDegrees)   // positive = the ball is to the left
                turn = (bearing * RobotConstants.POLLEN_TURN_GAIN)
                    .coerceIn(-RobotConstants.POLLEN_MAX_TURN, RobotConstants.POLLEN_MAX_TURN)
            }
        }

        var speed = 1.0
        if (gamepad1.right_bumper) {
            speed = RobotConstants.SLOW_MODE_SPEED
        }

        // Field-centric: turn the stick direction so it is measured from the drivers, not from the robot.
        val powers = ManualDrive.fieldCentric(
            forward * speed, strafe * speed, turn * speed, pose.heading(), driverHeading
        )
        drivetrain.drive(powers, true)

        // Fix odometry drift using a spot you can find by feel.
        // Two buttons, so it cannot happen by accident. (Read the dpad first: reading it uses up the press.)
        if (gamepad1.dpadDownWasPressed() && gamepad1.options) {
            localizer.setPose(Field.cornerPose(alliance))
            relocalizer.forgetOldPictures(System.nanoTime())   // older webcam pictures no longer match
            gamepad1.rumble(300)
        }
        if (gamepad1.shareWasPressed() && gamepad1.options) {
            localizer.setPose(pose.withHeading(driverHeading))
            relocalizer.forgetOldPictures(System.nanoTime())
            gamepad1.rumble(300)
        }
    }

    /** Moves our odometry position to (x, y). The heading is left alone: the Pinpoint measures it well. */
    private fun moveOdometryTo(position: Pose) {
        val device = pinpoint ?: return
        device.setPosX(position.x(), DistanceUnit.INCH)
        device.setPosY(position.y(), DistanceUnit.INCH)
    }

    private fun operatorButtons() {
        if (gamepad2.triangleWasPressed()) {
            shooter.isOn = !shooter.isOn
        }
        if (gamepad2.squareWasPressed()) {
            manualMode = !manualMode
        }

        // Pick the up CELL by hand: the one on the operator's left or right.
        if (gamepad2.dpadLeftWasPressed()) {
            hive.upCell = cellOnLeft
        }
        if (gamepad2.dpadRightWasPressed()) {
            hive.upCell = cellOnLeft.other()
        }

        // Shoot farther / shorter.
        if (gamepad2.dpadUpWasPressed()) {
            shooter.distanceTrimInches += 2.0
        }
        if (gamepad2.dpadDownWasPressed()) {
            shooter.distanceTrimInches -= 2.0
        }

        // Nudge the aim (positive = more to the left).
        if (gamepad2.leftBumperWasPressed()) {
            shooter.turretTrimDegrees += 1.0
        }
        if (gamepad2.rightBumperWasPressed()) {
            shooter.turretTrimDegrees -= 1.0
        }

        // Practice data logging: did the shot that is waiting for a label go in?
        // Only when exactly ONE shot is waiting, so a label can never go to the wrong ball.
        // (If balls went out too quickly, they are saved as UNLABELED instead.)
        val logger = shotLogger
        if (logger != null) {
            val wentIn = gamepad2.circleWasPressed()
            val missed = gamepad2.shareWasPressed()
            if (logger.waitingForLabel == 1) {
                if (wentIn) {
                    logger.label(ShotLogger.IN, matchTimer.seconds())
                } else if (missed) {
                    logger.label(ShotLogger.MISSED, matchTimer.seconds())
                }
            }
        }
    }

    /** One chain of if / else if, so each motor gets exactly one command per loop. */
    private fun intakeAndShoot() {
        // POLLEN only go into the flywheel while the shooter is ready.
        // Holding cross skips the wait for full speed, but never shoots from the wrong side (in auto-aim mode),
        // past the turret's limits, or with the flywheel off. In manual mode the operator checks the side.
        val canForce = gamepad2.cross && shooter.isOn && shooter.onOpenSide && !shooter.targetOutOfReach()
        val ready = shooter.isReady()
        val shooting = gamepad2.right_trigger_pressed && (ready || canForce)
        feeding = false
        forced = false

        if (gamepad1.left_trigger_pressed || gamepad2.left_trigger_pressed) {
            intake.eject()
            transfer.reverse()
        } else if (shooting) {
            intake.intake()
            transfer.feed()
            feeding = true
            forced = !ready
        } else if (gamepad1.right_trigger_pressed) {
            intake.intake()
            transfer.whileIntaking()
        } else {
            intake.stop()
            transfer.stop()
        }
    }

    /** Which CELL, in words the drivers use. */
    private fun cellName(cell: HiveCell): String {
        if (cell == cellOnLeft) return "LEFT"
        return "RIGHT"
    }

    private fun showTelemetry(pose: Pose) {
        // Most important first.
        telemetry.addData("Shooter", shooter.status())
        var target = "${cellName(hive.upCell)} CELL"
        val cameraCell = hive.cameraUpCell
        if (RobotConstants.USE_CAMERA_TO_FIND_UP_CELL && cameraCell != null && cameraCell != hive.upCell) {
            target += "   (webcam thinks ${cellName(cameraCell)})"
        }
        telemetry.addData("Target", target)
        telemetry.addData(
            "Aim",
            "%s  distance %.0f in (trim %+.0f), turret %.0f deg (trim %+.0f)".format(
                if (manualMode) "MANUAL" else "AUTO",
                shooter.distance, shooter.distanceTrimInches,
                shooter.turret.angleDegrees, shooter.turretTrimDegrees
            )
        )
        telemetry.addData(
            "Flywheel",
            "%.0f / %.0f rpm, hood %.2f".format(
                shooter.flywheel.currentRpm(), shooter.flywheel.targetRpm, shooter.hood.position
            )
        )
        telemetry.addData(
            "Position",
            "x %.1f  y %.1f  heading %.0f".format(pose.x(), pose.y(), Math.toDegrees(pose.heading()))
        )
        val logger = shotLogger
        if (logger != null) {
            telemetry.addData(
                "Shot log",
                "%d shots, %d waiting (one at a time: circle = in, SHARE = missed)".format(logger.shotsFound, logger.waitingForLabel)
            )
        }
        val secondsLeft = (RobotConstants.TELEOP_SECONDS - matchTimer.seconds()).coerceAtLeast(0.0)
        if (secondsLeft > 0.0) {
            telemetry.addData("Time left", "%d:%02d".format(secondsLeft.toInt() / 60, secondsLeft.toInt() % 60))
        } else {
            telemetry.addLine("TELEOP is over: press STOP on the Driver Station.")
        }
        telemetry.update()
    }
}

@TeleOp(name = RobotConstants.TELEOP_RED, group = "BioBuzz")
class TeleOpRed : BioBuzzTeleOp(Alliance.RED)

@TeleOp(name = RobotConstants.TELEOP_BLUE, group = "BioBuzz")
class TeleOpBlue : BioBuzzTeleOp(Alliance.BLUE)
