package org.firstinspires.ftc.teamcode.tools

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.RobotConstants
import org.firstinspires.ftc.teamcode.control.motor
import kotlin.math.abs

class Flywheel(
    hardwareMap: HardwareMap,
    name: String = RobotConstants.FLYWHEEL_MOTOR,
    direction: DcMotorSimple.Direction =
        if (RobotConstants.FLYWHEEL_REVERSED) DcMotorSimple.Direction.REVERSE else DcMotorSimple.Direction.FORWARD
) : motor(hardwareMap, name, direction) {

    /** The speed we asked for, in RPM (0 = off). */
    var targetRpm = 0.0
        private set

    init {
        // FLOAT lets the flywheel coast to a stop instead of braking hard.
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        // The motor's own speed control (RUN_USING_ENCODER) holds a speed even as the battery drains.
        motor.setVelocityPIDFCoefficients(
            RobotConstants.FLYWHEEL_P,
            RobotConstants.FLYWHEEL_I,
            RobotConstants.FLYWHEEL_D,
            RobotConstants.FLYWHEEL_F
        )
    }

    /** Spin at this many RPM. 0 (or less) turns the flywheel off and lets it coast. */
    fun setRpm(rpm: Double) {
        if (rpm <= 0.0) {
            targetRpm = 0.0
            // With speed control on, power 0 means "hold speed 0" and brakes the flywheel hard.
            // Turning speed control off first lets it coast down gently.
            motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
            motor.power = 0.0
            return
        }
        targetRpm = rpm
        // The motor wants ticks per second: RPM * ticks per turn / 60 seconds.
        // Setting a velocity turns speed control (RUN_USING_ENCODER) back on by itself.
        motor.velocity = rpm * RobotConstants.FLYWHEEL_TICKS_PER_REV / 60.0
    }

    /** How fast the flywheel is really spinning, in RPM. */
    fun currentRpm(): Double {
        return motor.velocity * 60.0 / RobotConstants.FLYWHEEL_TICKS_PER_REV
    }

    /** True when the flywheel is on and close enough to its target speed to shoot. */
    fun isAtSpeed(): Boolean {
        if (targetRpm <= 0.0) return false
        return abs(currentRpm() - targetRpm) < RobotConstants.FLYWHEEL_RPM_TOLERANCE
    }
}
