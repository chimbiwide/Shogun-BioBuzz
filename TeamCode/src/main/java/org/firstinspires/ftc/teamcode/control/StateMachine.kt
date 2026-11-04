package org.firstinspires.ftc.teamcode.control

import com.pedropathing.follower.ManualDrive
import com.pedropathing.math.Pose
import com.pedropathing.revhub.drivetrains.Mecanum
import org.firstinspires.ftc.teamcode.datatypes.STATE
import org.firstinspires.ftc.teamcode.tools.Intake
import org.firstinspires.ftc.teamcode.tools.Shooter
import org.firstinspires.ftc.teamcode.tools.Transfer
import org.firstinspires.ftc.teamcode.tools.Turret

class StateMachine(
    var drivetrain: Mecanum,
    var intake: Intake,
    var transfer: Transfer,
    var shooter: Shooter,
    var turret: Turret

) {
    var state: STATE = STATE.IDLE

    fun runState(state: STATE) {
        when (state) {

            STATE.IDLE -> {
                intake.stop()
                drivetrain.stop()
                transfer.stop()
                shooter.flywheel.stop()
            }
            STATE.DRIVE -> {}
            STATE.INTAKE -> {}
            STATE.EJECT -> {}
            STATE.AIM -> {}
            STATE.SHOOT -> {}
        }
    }

    private fun drive(forward: Double,
                      strafe: Double,
                      turn: Double,
                      speed: Double,
                      pose: Pose) {
        val powers = ManualDrive.fieldCentric(
            forward * speed, strafe * speed, turn * speed, pose.heading(), 0.0
        )
        drivetrain.drive(powers, true)
    }
}