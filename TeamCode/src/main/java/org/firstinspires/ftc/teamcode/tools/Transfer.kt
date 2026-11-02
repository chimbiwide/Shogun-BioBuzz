package org.firstinspires.ftc.teamcode.tools

import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.RobotConstants
import org.firstinspires.ftc.teamcode.control.motor

/** The transfer motor: moves POLLEN from the intake up to the flywheel. */
class Transfer(
    hardwareMap: HardwareMap,
    name: String = RobotConstants.TRANSFER_MOTOR,
    direction: DcMotorSimple.Direction = DcMotorSimple.Direction.REVERSE
) : motor(hardwareMap, name, direction) {

    /** Push POLLEN up into the flywheel (this shoots them). */
    fun feed() {
        run(RobotConstants.TRANSFER_FEED_POWER)
    }

    /** Run backwards to clear a jam. */
    fun reverse() {
        run(RobotConstants.TRANSFER_REVERSE_POWER)
    }

    /** Speed used while the intake is running (see TRANSFER_POWER_WHILE_INTAKING). */
    fun whileIntaking() {
        run(RobotConstants.TRANSFER_POWER_WHILE_INTAKING)
    }
}
