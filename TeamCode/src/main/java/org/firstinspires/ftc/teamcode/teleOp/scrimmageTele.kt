package org.firstinspires.ftc.teamcode.teleOp

import com.pedropathing.drivetrain.DrivePowers
import com.pedropathing.follower.Follower
import com.pedropathing.follower.ManualDrive
import com.pedropathing.math.Pose
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.pedro.Constants

@TeleOp(name = "Scrimmage TeleOp")
class scrimmageTele : OpMode() {
    private lateinit var follower: Follower

    override fun init() {
        follower = Constants.create(hardwareMap)
    }

    override fun loop() {

        val powers: DrivePowers = ManualDrive.fieldCentric(
            -gamepad1.left_stick_y.toDouble(),
            gamepad1.left_stick_x.toDouble(),
            gamepad1.right_stick_x.toDouble(),
            follower.pose().heading()
        )
        follower.manual(powers)
        follower.update()

        val robotPose: Pose = follower.pose()

        telemetry.addData("X: ", robotPose.x())
        telemetry.addData("Y: ", robotPose.y())
        telemetry.addData("Heading: ", robotPose.heading())
    }
}