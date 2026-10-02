package org.firstinspires.ftc.teamcode.teleOp

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.tools.Transfer
import kotlin.reflect.KClass


@TeleOp(name = "teleOp transfer")
class TransferOp() : OpMode() {
    private lateinit var transfer: Transfer
    override fun init() {
        transfer = Transfer(hardwareMap, "transfer")
    }


    override fun loop() {
        if(gamepad1.square){
            transfer.run()
        }
    }


}