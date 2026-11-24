package org.firstinspires.ftc.teamcode.teleOp

import androidx.core.graphics.component1
import com.pedropathing.follower.Follower
import com.pedropathing.math.Pose
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.pedro.Constants
import org.firstinspires.ftc.teamcode.tools.Intake
import org.firstinspires.ftc.teamcode.tools.Shooter
import org.firstinspires.ftc.teamcode.tools.Transfer
import org.firstinspires.ftc.teamcode.tools.Turret

@TeleOp(name = "Test TeleOp")
class testTele : OpMode() {
    private lateinit var follower: Follower
    private lateinit var intake: Intake
    private lateinit var transfer: Transfer
    private lateinit var shooter: Shooter
    private var rpm: Double = 500.0
    private var turretPos = 0.0
    private var hoodPos = 0.9

    override fun init() {
        for (hub in hardwareMap.getAll(LynxModule::class.java)) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.AUTO
        }
        follower = Constants.create(hardwareMap)
        intake = Intake(hardwareMap)
        transfer = Transfer(hardwareMap)
        shooter = Shooter(hardwareMap)
    }

    override fun start() {
       follower.setPose(Pose.zero())
    }

    override fun loop() {
        follower.update()
        val pose: Pose = follower.pose()

        drive(follower)

        setRpm()
        aim()
        aimHood()
        intakeAndShoot()

        showTele(pose)
    }

    override fun stop() {
        shooter.isOn = false
        shooter.flywheel.setRpm(0.0)
        intake.stop()
        transfer.stop()
    }

    private fun aim() {
        if (gamepad1.right_bumper) {
            turretPos += 0.03
        }
        else if (gamepad1.left_bumper) {
            turretPos -= 0.03
        }
        turretPos = turretPos.coerceIn(0.0,1.0)
        shooter.turret.setServoPosition(turretPos)
    }

    private fun aimHood() {
        if (gamepad1.dpad_up) {
            hoodPos -= 0.01
        }
        else if (gamepad1.dpad_down) {
            hoodPos += 0.01
        }
        hoodPos = hoodPos.coerceIn(0.0, 1.0)
        shooter.hood.setServoPosition(hoodPos)
    }

    private fun setRpm() {
        if (gamepad1.triangleWasPressed()) {
            rpm += 50.0
        }
        if (gamepad1.crossWasPressed()) {
            rpm -= 50.0
        }
        shooter.flywheel.setRpm(rpm)
    }

    private fun intakeAndShoot() {
        val unJam = gamepad1.left_trigger_pressed
        val wantsToShoot = gamepad1.square

        if (unJam) {
            intake.eject()
            transfer.reverse()
        } else if (wantsToShoot) {
            transfer.feed()
        }  else if (gamepad1.right_trigger_pressed) {
            intake.intake()
            transfer.whileIntaking()
        } else {
            intake.stop()
            transfer.stop()
        }
    }

    private fun drive(follower: Follower) {
        val forward = -gamepad1.left_stick_y.toDouble()
        val lateral = gamepad1.left_stick_x.toDouble()
        val yaw = -gamepad1.right_stick_x.toDouble()
        follower.manual(forward, lateral, yaw)
    }

    private fun showTele(pose: Pose) {
        telemetry.addData(
            "Flywheel",
            "%.1f / %.1f rpm, hood %.2f".format(
                shooter.flywheel.currentRpm(),
                shooter.flywheel.targetRpm,
                shooter.hood.position
            )
        )
        telemetry.addData(
            "Position",
            "x %.2f y %.2f heading %.2f".format(
                pose.x(),
                pose.y(),
                Math.toDegrees(pose.heading())
            )
        )
        telemetry.addData("Turret Pos", turretPos)
        telemetry.update()
    }
}