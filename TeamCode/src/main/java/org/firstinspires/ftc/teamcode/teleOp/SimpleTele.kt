package org.firstinspires.ftc.teamcode.teleOp

import com.pedropathing.follower.ManualDrive
import com.pedropathing.math.Pose
import com.pedropathing.revhub.drivetrains.Mecanum
import com.pedropathing.revhub.localizers.PinpointLocalizer
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.RobotConstants
import org.firstinspires.ftc.teamcode.control.StateMachine
import org.firstinspires.ftc.teamcode.datatypes.STATE
import org.firstinspires.ftc.teamcode.pedro.Constants
import org.firstinspires.ftc.teamcode.tools.Intake
import org.firstinspires.ftc.teamcode.tools.Shooter
import org.firstinspires.ftc.teamcode.tools.Transfer
import org.firstinspires.ftc.teamcode.tools.Turret

/*
 * DRIVER (gamepad 1)
 *   left stick ........ drive
 *   right stick X ..... turn
 *   right bumper ...... hold for slow mode
 *   right trigger ..... intake
 *
 * Before START, point the robot's intake straight away from the drivers.
 *
 * SHOOTING (gamepad 1 OR gamepad 2, both work)
 *   triangle .......... flywheel on / off
 *   cross ............. hold to shoot (POLLEN only feed in once the flywheel is up to speed)
 *   dpad up / down .... shots go farther / shorter
 *   left trigger ...... un-jam (intake + transfer backwards)
 *
 */
@TeleOp(name = "Simple TeleOp", group = "BioBuzz")
class SimpleTele : OpMode() {
    private lateinit var drivetrain: Mecanum
    private lateinit var localizer: PinpointLocalizer
    private lateinit var intake: Intake
    private lateinit var transfer: Transfer
    private lateinit var shooter: Shooter
    private lateinit var turret: Turret

    // Never use "runtime" for matchtime: it starts counting at INIT, not at START.
    private val matchTimer = ElapsedTime()
    private var flywheelStoppedAtEnd = false
    private lateinit var machine: StateMachine
    private lateinit var states: STATE

    override fun init() {
        for (hub in hardwareMap.getAll(LynxModule::class.java)) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.AUTO
        }

        drivetrain = Mecanum(hardwareMap, Constants.drivetrainConfig)
        localizer = PinpointLocalizer(hardwareMap, Constants.localizerConfig)
        intake = Intake(hardwareMap)
        transfer = Transfer(hardwareMap)
        shooter = Shooter(hardwareMap)
        turret = Turret(hardwareMap)
        states = STATE.IDLE
    }

    override fun init_loop() {
        telemetry.addLine("Simple TeleOp: no cameras, turret straight ahead.")
        telemetry.update()
    }

    override fun start() {
        matchTimer.reset()
        localizer.setPose(Pose.zero())
        machine = StateMachine(
            drivetrain,
            intake,
            transfer,
            shooter,
            turret
        )

    }

    override fun loop() {
        localizer.update()
        val pose = localizer.pose()

        var turretPos = 0.0


        machine.runState(states)
        drive(pose)

        shooterButtons()

        if (gamepad1.dpadLeftWasPressed()) {
            turretPos -= 0.05
            turret.setServoPosition(turretPos)
        }
        if (gamepad1.dpadRightWasPressed()) {
            turretPos += 0.05
            turret.setServoPosition(turretPos)
        }


        shooter.aimStraight(RobotConstants.MANUAL_DISTANCE_INCHES)
        intakeAndShoot()

        showTelemetry(pose, turretPos)
    }

    override fun stop() {
        shooter.isOn = false
        shooter.flywheel.setRpm(0.0)
        intake.stop()
        transfer.stop()
        drivetrain.stop()
    }

    private fun drive(pose: Pose) {
        val forward = -gamepad1.left_stick_y.toDouble()
        val strafe = -gamepad1.left_stick_x.toDouble()
        val turn = -gamepad1.right_stick_x.toDouble()

        var speed = 1.0
        if (gamepad1.right_bumper) {
            speed = RobotConstants.SLOW_MODE_SPEED
        }

        val powers = ManualDrive.fieldCentric(
            forward * speed, strafe * speed, turn * speed, pose.heading(), 0.0
        )
        drivetrain.drive(powers, true)

        if (gamepad1.shareWasPressed() && gamepad1.options) {
            localizer.setPose(pose.withHeading(0.0))
            gamepad1.rumble(300)
        }
    }

    private fun shooterButtons() {
        if (gamepad1.triangleWasPressed() || gamepad2.triangleWasPressed()) {
            shooter.isOn = !shooter.isOn
        }

        if (gamepad1.dpadUpWasPressed() || gamepad2.dpadUpWasPressed()) {
            shooter.distanceTrimInches += 2.0
        }
        if (gamepad1.dpadDownWasPressed() || gamepad2.dpadDownWasPressed()) {
            shooter.distanceTrimInches -= 2.0
        }
    }

    /** One chain of if / else if, so each motor gets exactly one command per loop. */
    private fun intakeAndShoot() {
        val unJam = gamepad1.left_trigger_pressed || gamepad2.left_trigger_pressed
        val wantsToShoot = gamepad1.cross || gamepad2.cross

        if (unJam) {
            intake.eject()
            transfer.reverse()
        } else if (wantsToShoot) {
            // POLLEN only go into the flywheel once it is up to speed, so every shot flies the same.
            intake.intake()
            transfer.feed()
        } else if (gamepad1.right_trigger_pressed) {
            intake.intake()
            transfer.reverse()
        } else {
            intake.stop()
            transfer.stop()
        }
    }

    private fun showTelemetry(pose: Pose, turretPos: Double) {
        // Most important first.
        telemetry.addData("Shooter", shooter.status())
        telemetry.addData(
            "Flywheel",
            "%.0f / %.0f rpm, hood %.2f".format(
                shooter.flywheel.currentRpm(), shooter.flywheel.targetRpm, shooter.hood.position
            )
        )
        telemetry.addData(
            "Shot distance",
            "%.0f in (trim %+.0f)".format(shooter.distance, shooter.distanceTrimInches)
        )
        telemetry.addData(
            "Position",
            "x %.1f  y %.1f  heading %.0f".format(pose.x(), pose.y(), Math.toDegrees(pose.heading()))
        )
        val secondsLeft = (RobotConstants.TELEOP_SECONDS - matchTimer.seconds()).coerceAtLeast(0.0)
        if (secondsLeft > 0.0) {
            telemetry.addData("Time left", "%d:%02d".format(secondsLeft.toInt() / 60, secondsLeft.toInt() % 60))
        } else {
            telemetry.addLine("TELEOP is over: press STOP on the Driver Station.")
        }
        telemetry.addData("Turret Pos", turretPos)
        telemetry.update()
    }
}
