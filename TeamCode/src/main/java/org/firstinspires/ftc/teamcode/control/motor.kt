package org.firstinspires.ftc.teamcode.control

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap

abstract class motor(
    hardwareMap: HardwareMap,
    name: String,
    direction: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD
) {
    // DcMotorEx is a DcMotor that can also do velocity control (the flywheel needs that).
    val motor: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, name)
    val isBusy: Boolean
        get() = motor.isBusy

    init {
        motor.direction = direction
    }

    fun run(power: Double = 1.0) {
        motor.power = power
    }

    fun stop() {
        motor.power = 0.0
    }
}
