package org.firstinspires.ftc.teamcode.tools

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap

class Intake(hardwareMap: HardwareMap) {
    val intake: DcMotor = hardwareMap.get(DcMotor::class.java, "intake")



}