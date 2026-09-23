package org.firstinspires.ftc.teamcode.teleOp

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor

@TeleOp(name = "teleOp flywheel")
class flywheel : OpMode(){
    private lateinit var flyWheel: DcMotor
    override fun init() {
        flyWheel = hardwareMap.get(DcMotor::class.java, "intake")
    }

    override fun loop() {
        if(gamepad1.circle){
            flyWheel.power = 1.0
        }
    }


}
