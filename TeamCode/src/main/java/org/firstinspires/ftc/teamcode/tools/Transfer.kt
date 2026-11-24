package org.firstinspires.ftc.teamcode.tools

import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.RobotConstants
import org.firstinspires.ftc.teamcode.control.motor

class Transfer(
    hardwareMap: HardwareMap,
    name: String = RobotConstants.TRANSFER_MOTOR,
    direction: DcMotorSimple.Direction = DcMotorSimple.Direction.REVERSE
) : motor(hardwareMap, name, direction) {

    fun feed() {
        run(RobotConstants.TRANSFER_FEED_POWER)
    }

    fun reverse() {
        run(RobotConstants.TRANSFER_REVERSE_POWER)
    }

    fun whileIntaking() {
        run(RobotConstants.TRANSFER_POWER_WHILE_INTAKING)
    }
}
