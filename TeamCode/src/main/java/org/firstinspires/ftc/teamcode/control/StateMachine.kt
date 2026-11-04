package org.firstinspires.ftc.teamcode.control

import com.pedropathing.revhub.drivetrains.Mecanum
import org.firstinspires.ftc.teamcode.datatypes.STATE
import org.firstinspires.ftc.teamcode.tools.Intake
import org.firstinspires.ftc.teamcode.tools.Shooter
import org.firstinspires.ftc.teamcode.tools.Transfer
import org.firstinspires.ftc.teamcode.tools.Turret

class StateMachine(
    drivetrain: Mecanum,
    intake: Intake,
    transfer: Transfer,
    shooter: Shooter,
    turret: Turret
) {
    var state: STATE = STATE.IDLE

    fun runState(state: STATE) {
        when (state) {
            STATE.IDLE -> {}
            STATE.DRIVE -> {}
            STATE.INTAKE -> {}
            STATE.EJECT -> {}
            STATE.AIM -> {}
            STATE.SHOOT -> {}
        }
    }
}