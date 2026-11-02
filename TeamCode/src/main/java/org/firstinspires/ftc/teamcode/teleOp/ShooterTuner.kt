package org.firstinspires.ftc.teamcode.teleOp

import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.drivetrain.DrivePowers
import com.pedropathing.math.Pose
import com.pedropathing.revhub.drivetrains.Mecanum
import com.pedropathing.revhub.localizers.PinpointLocalizer
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.RobotConstants
import org.firstinspires.ftc.teamcode.game.Alliance
import org.firstinspires.ftc.teamcode.game.Field
import org.firstinspires.ftc.teamcode.game.HiveCell
import org.firstinspires.ftc.teamcode.game.Relocalizer
import org.firstinspires.ftc.teamcode.pedro.Constants
import org.firstinspires.ftc.teamcode.tools.Intake
import org.firstinspires.ftc.teamcode.tools.LimeLight
import org.firstinspires.ftc.teamcode.tools.Shooter
import org.firstinspires.ftc.teamcode.tools.Transfer
import org.firstinspires.ftc.teamcode.tools.Webcam
import kotlin.math.hypot

/*
 * Shooter Tuner: finds the numbers for RobotConstants. Uses the RED half of the field.
 *
 * GAMEPAD 1
 *   left stick / right stick X ... drive (robot-centric)
 *   OPTIONS ...................... robot is in the RED corner: back on the red wall, left side on the far wall.
 *                                  Press to reset the position there. (The OpMode also starts there.)
 *   triangle ..................... switch the target CELL (audience side / far side)
 *   cross ........................ turret: manual servo position  <->  auto-aim using the calibration
 *   dpad left / right ............ turret servo position -/+ 0.01 (manual turret)
 *   dpad up / down ............... hood servo position +/- 0.01 (no limits here: be careful)
 *   right bumper / left bumper ... flywheel +/- 100 RPM
 *   circle ....................... flywheel on / off
 *   right trigger ................ feed POLLEN (shoot)
 *   left trigger ................. un-jam
 * GAMEPAD 2
 *   cross (hold) ................. run only the transfer (feed), without the intake
 *
 * HOW TO TUNE (in this order)
 *  0. Odometry (before anything else): on open floor, spin in place 3 or 4 turns with the right stick and
 *     watch "Position" x and y. They must stay within about 1 inch the whole time. If they wander more,
 *     the pod offsets in pedro/Constants.java are wrong. Measure each pod from the robot's center:
 *     xPodOffset = how far the forward pod is LEFT of center, yPodOffset = how far the strafe pod is
 *     FORWARD of center (right and back are negative). Or run Pedro's Pinpoint tuner again, turning the
 *     robot by hand exactly 180 degrees about its center. Then check again.
 *  1. Hood: move it slowly to each end until it stops. Set HOOD_MIN/MAX_POSITION a little inside those.
 *     Then put the hood at HOOD_MIN_POSITION and at HOOD_MAX_POSITION and measure its angle each time
 *     (a protractor or a phone level app): HOOD_DEGREES_AT_MIN/MAX_POSITION. The shot logger uses them.
 *  2. Turret: with the manual turret, point the shooter straight forward and copy "Turret servo" into
 *     TURRET_POSITION_AT_0_DEGREES. Turn it to the LEFT by an angle you can measure (for example 30 degrees
 *     with a protractor): put that angle in TURRET_CALIBRATION_DEGREES and the servo number in
 *     TURRET_POSITION_AT_CALIBRATION. Write down how far it can safely turn each way (TURRET_MIN/MAX_DEGREES).
 *     Re-deploy, press cross for auto-aim, and drive around: the shooter should keep pointing at the CELL.
 *     Turret speed: press OPTIONS (the robot now thinks it is in the start corner, where both CELLS are far
 *     to its right). Turn the manual turret all the way to its LEFT limit, then press cross: auto-aim swings
 *     it to its RIGHT limit. Time the swing (a slow-motion phone video helps), then
 *     TURRET_DEGREES_PER_SECOND = (TURRET_MAX_DEGREES - TURRET_MIN_DEGREES) / seconds.
 *  3. Transfer: hold gamepad 2 cross. The transfer must move POLLEN UP toward the flywheel. If it moves them
 *     down, swap the signs of TRANSFER_FEED_POWER and TRANSFER_REVERSE_POWER.
 *  4. Shot table: press triangle until "Target CELL" is the CELL that is up. Park in front of its opening,
 *     change RPM and hood until POLLEN go in every time, then copy "distance, RPM, hood" as a row in
 *     SHOT_TABLE. Repeat at several distances.
 *  5. Webcam tag rows: measure the webcam mount (WEBCAM_* in RobotConstants) first. Face the RED HIVE from a
 *     few spots. Each "Tag row" line must read about 50 inches high under the up CELL and about 36 under the
 *     down CELL (within about 2 inches). If not, re-measure the webcam's height and tilt.
 *     Then you may turn on USE_CAMERA_TO_FIND_UP_CELL.
 *  6. Webcam position: press OPTIONS with the robot really in the start corner, drive slowly to a few spots
 *     where the webcam sees the tags, stop, and compare "Camera position" with "Position". They should agree
 *     within about 1 inch. Then you may turn on USE_TAGS_TO_FIX_POSITION.
 *  7. Limelight: re-aim it about 25 degrees down and measure LIMELIGHT_*. Put a POLLEN on the floor 24 inches in
 *     front of the ROBOT CENTER (the same center the pod offsets use, not the lens): "Nearest POLLEN" must say
 *     about 24 in ahead. Try 36 and 48 inches, and one ball off to the side, too.
 *     (If the distance is off, fix LIMELIGHT_PITCH_DEGREES first: one degree moves it about 1 inch at 24 inches.)
 */
@TeleOp(name = "Shooter Tuner", group = "BioBuzz Tuning")
class ShooterTuner : OpMode() {
    private lateinit var drivetrain: Mecanum
    private lateinit var localizer: PinpointLocalizer
    private lateinit var intake: Intake
    private lateinit var transfer: Transfer
    private lateinit var shooter: Shooter
    private lateinit var limelight: LimeLight
    private var webcam: Webcam? = null
    private val relocalizer = Relocalizer()

    private var targetCell = HiveCell.AUDIENCE_SIDE
    private var autoAim = false
    private var turretPosition = RobotConstants.TURRET_POSITION_AT_0_DEGREES
    private var hoodPosition = (RobotConstants.HOOD_MIN_POSITION + RobotConstants.HOOD_MAX_POSITION) / 2.0
    private var rpm = 3000.0
    private var flywheelOn = false

    override fun init() {
        // Telemetry also goes to Panels, but only when Panels is turned on (see RobotConstants.USE_PANELS).
        if (RobotConstants.USE_PANELS) {
            telemetry = JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry)
        }
        drivetrain = Mecanum(hardwareMap, Constants.drivetrainConfig)
        localizer = PinpointLocalizer(hardwareMap, Constants.localizerConfig)
        intake = Intake(hardwareMap)
        transfer = Transfer(hardwareMap)
        shooter = Shooter(hardwareMap)
        limelight = LimeLight(hardwareMap)
        if (RobotConstants.USE_WEBCAM) {
            webcam = Webcam(hardwareMap)
        }
    }

    override fun init_loop() {
        webcam?.applyCameraSettings()   // short exposure, once the camera is streaming
        telemetry.addData("Webcam", webcam?.status() ?: "not used (USE_WEBCAM = false)")
        telemetry.addData("Limelight", if (limelight.isConnected()) "answering" else "NOT ANSWERING")
        telemetry.update()
    }

    override fun start() {
        localizer.setPose(Field.cornerPose(Alliance.RED))
    }

    override fun loop() {
        localizer.update()
        val pose = localizer.pose()

        // Drive (robot-centric, so it is easy to make small moves).
        val powers = DrivePowers(
            -gamepad1.left_stick_y.toDouble(),
            -gamepad1.left_stick_x.toDouble(),
            -gamepad1.right_stick_x.toDouble()
        )
        drivetrain.drive(powers, true)

        if (gamepad1.optionsWasPressed()) {
            localizer.setPose(Field.cornerPose(Alliance.RED))
        }
        if (gamepad1.triangleWasPressed()) {
            targetCell = targetCell.other()
        }
        if (gamepad1.crossWasPressed()) {
            autoAim = !autoAim
        }
        if (gamepad1.dpadLeftWasPressed()) {
            turretPosition -= 0.01
        }
        if (gamepad1.dpadRightWasPressed()) {
            turretPosition += 0.01
        }
        if (gamepad1.dpadUpWasPressed()) {
            hoodPosition += 0.01
        }
        if (gamepad1.dpadDownWasPressed()) {
            hoodPosition -= 0.01
        }
        if (gamepad1.rightBumperWasPressed()) {
            rpm += 100.0
        }
        if (gamepad1.leftBumperWasPressed()) {
            rpm -= 100.0
        }
        if (gamepad1.circleWasPressed()) {
            flywheelOn = !flywheelOn
        }
        turretPosition = turretPosition.coerceIn(0.0, 1.0)
        hoodPosition = hoodPosition.coerceIn(0.0, 1.0)

        val target = Field.openingPosition(Alliance.RED, targetCell)
        if (autoAim) {
            shooter.pointTurretAt(pose, target)
        } else {
            shooter.turret.setServoPosition(turretPosition)
        }
        shooter.hood.setServoPosition(hoodPosition)
        if (flywheelOn) {
            shooter.flywheel.setRpm(rpm)
        } else {
            shooter.flywheel.setRpm(0.0)
        }

        // One chain of if / else if, so the intake and the transfer each get one command per loop.
        if (gamepad2.cross) {
            intake.stop()
            transfer.feed()
        } else if (gamepad1.left_trigger_pressed) {
            intake.eject()
            transfer.reverse()
        } else if (gamepad1.right_trigger_pressed) {
            intake.intake()
            transfer.feed()
        } else {
            intake.stop()
            transfer.stop()
        }

        telemetry.addData("Target CELL", "RED $targetCell")
        telemetry.addData("distance", "%.0f in".format(shooter.distanceTo(pose, target)))
        telemetry.addData("RPM", "%.0f  (now %.0f)".format(rpm, shooter.flywheel.currentRpm()))
        telemetry.addData("hood", "%.2f  (%.1f degrees, once measured)".format(shooter.hood.position, shooter.hood.angleDegrees()))
        telemetry.addData("Turret mode", if (autoAim) "AUTO-AIM" else "manual")
        telemetry.addData("Turret servo", "%.3f".format(shooter.turret.servoPosition))
        if (autoAim) {
            telemetry.addData("Turret angle", "%.0f deg".format(shooter.turret.angleDegrees))
            if (shooter.targetOutOfReach()) {
                telemetry.addLine("Target is past the turret's limits: turn the robot")
            }
        }
        telemetry.addData(
            "Position",
            "x %.1f  y %.1f  heading %.0f".format(pose.x(), pose.y(), Math.toDegrees(pose.heading()))
        )
        showCameras(pose)
        telemetry.update()
    }

    /** What the webcam (AprilTags) and the Limelight (POLLEN) see, to check their numbers (steps 5 to 7). */
    private fun showCameras(pose: Pose) {
        val camera = webcam
        if (camera == null) {
            telemetry.addData("Webcam", "not used (USE_WEBCAM = false)")
        } else {
            camera.applyCameraSettings()   // does nothing once the exposure is set
            camera.update()
            if (camera.latestSightings.isEmpty()) {
                telemetry.addData("Tag rows", "none in view (webcam %s)".format(camera.status()))
            }
            for (row in camera.latestSightings) {
                telemetry.addData(
                    "Tag row ${row.alliance} ${row.cell}",
                    "%.1f in high, so %s.  %.0f in away, %d%% of its tags seen".format(
                        row.seen.up, if (row.cellIsUp) "UP" else "down",
                        hypot(row.seen.forward, row.seen.left), row.percentFound
                    )
                )
            }
            val fromTags = relocalizer.positionFromTags(camera.latestSightings, pose.heading(), System.nanoTime())
            if (fromTags != null) {
                telemetry.addData("Camera position", "x %.1f  y %.1f".format(fromTags.x(), fromTags.y()))
            }
        }

        if (!limelight.isConnected()) {
            telemetry.addData("Limelight", "NOT ANSWERING (missing, unplugged or still starting up)")
        } else {
            val balls = limelight.pollenInView(pose)
            telemetry.addData("Limelight", "pipeline %s, %d POLLEN in view".format(limelight.pipelineIndex(), balls.size))
            val nearest = balls.minByOrNull { it.distanceInches }
            if (nearest != null) {
                telemetry.addData(
                    "Nearest POLLEN",
                    "%.1f in ahead, %.1f in left of the robot center (turn %.0f deg)".format(
                        nearest.seen.forward, nearest.seen.left, nearest.bearingDegrees
                    )
                )
            }
        }
    }

    override fun stop() {
        shooter.flywheel.setRpm(0.0)
        intake.stop()
        transfer.stop()
        drivetrain.stop()
        limelight.stop()
    }
}
