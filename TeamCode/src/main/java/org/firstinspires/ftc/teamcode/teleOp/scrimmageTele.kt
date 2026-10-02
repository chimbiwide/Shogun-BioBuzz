package org.firstinspires.ftc.teamcode.teleOp

import com.pedropathing.follower.Follower
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

        var forward: Double = (-gamepad1.left_stick_y).toDouble()
        var lateral: Double = (gamepad1.left_stick_x).toDouble()
        var turn: Double = (gamepad1.right_stick_x).toDouble()

        follower.manual(forward, lateral, turn)
        follower.update()
    }
}