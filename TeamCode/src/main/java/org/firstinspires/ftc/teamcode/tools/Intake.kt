package org.firstinspires.ftc.teamcode.tools

import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.RobotConstants
import org.firstinspires.ftc.teamcode.control.motor

class Intake(
    hardwareMap: HardwareMap,
    name: String = RobotConstants.INTAKE_MOTOR,
    direction: DcMotorSimple.Direction = DcMotorSimple.Direction.FORWARD
) : motor(hardwareMap, name, direction) {

    /** Pull POLLEN into the robot. */
    fun intake() {
        run(RobotConstants.INTAKE_POWER)
    }

    /** Push POLLEN back out, to clear a jam or drop an extra one. */
    fun eject() {
        run(RobotConstants.EJECT_POWER)
    }
}
