package org.firstinspires.ftc.teamcode.test

import com.pedropathing.follower.Follower
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.pedro.Constants

@Autonomous(name = "Servo Zero")
class `zero-servo`: LinearOpMode() {
    private lateinit var servo1: Servo;
    private lateinit var servo2: Servo;

    override fun runOpMode() {
        servo1 = hardwareMap.get(Servo::class.java, "servo1" )
        servo2 = hardwareMap.get(Servo::class.java, "servo2" )
        waitForStart()

        servo1.position = 0.0
        servo2.position = 0.0
        //This is ABA 120-0 Hex Code combo

    }
}