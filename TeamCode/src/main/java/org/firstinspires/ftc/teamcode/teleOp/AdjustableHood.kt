package org.firstinspires.ftc.teamcode.teleOp

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo

@TeleOp(name = "AdjustableHood")
class AdjustableHood : OpMode(){
    private lateinit var transfer : Servo
    override fun init() {
        transfer = hardwareMap.get(Servo::class.java, "hood")
    }

    override fun loop() {
        when(){
            gamepad1.right_bumper -> transfer.position = 0.5
            gamepad1.left_bumper -> transfer.position = -0.5

        }
    }
}