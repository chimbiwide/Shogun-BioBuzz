package org.firstinspires.ftc.teamcode.tools

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.RobotConstants
import kotlin.math.abs

/**
 * The flywheel: two motors on one shaft. Our own speed control (PIDF) reads flywheel1's encoder
 * and gives both motors the same power. setRpm() runs one step of it, so call setRpm() EVERY loop
 * (Shooter.aimAt() / aimStraight() already do).
 */
class Flywheel(
    hardwareMap: HardwareMap,
    flywheel1: String = RobotConstants.FLYWHEEL_MOTOR_1,
    flywheel2: String = RobotConstants.FLYWHEEL_MOTOR_2,
    direction: DcMotorSimple.Direction =
        if (RobotConstants.FLYWHEEL_REVERSED) DcMotorSimple.Direction.REVERSE else DcMotorSimple.Direction.FORWARD
) {

    /** The speed we asked for, in RPM (0 = off). */
    var targetRpm = 0.0
        private set

    val motor1: DcMotorEx = hardwareMap.get(DcMotorEx::class.java,flywheel1)
    val motor2: DcMotorEx = hardwareMap.get(DcMotorEx::class.java,flywheel2)

    private val stepTimer = ElapsedTime()
    private var integral = 0.0
    private var lastError = 0.0

    init {
        motor1.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        motor2.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT

        motor1.direction = direction
        motor2.direction = direction

        // We set the power ourselves (see setRpm). The encoder still measures the speed in this mode.
        motor1.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motor2.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    /** Spin at this many RPM: one step of the speed control. 0 (or less) turns the flywheel off and lets it coast. */
    fun setRpm(rpm: Double) {
        if (rpm <= 0.0) {
            targetRpm = 0.0
            integral = 0.0
            lastError = 0.0
            motor1.power = 0.0
            motor2.power = 0.0
            return
        }
        targetRpm = rpm

        // Time since the last step: one loop. On the first step (or after a long pause) use a normal loop time.
        var dt = stepTimer.seconds()
        stepTimer.reset()
        if (dt <= 0.0 || dt > 0.1) {
            dt = 0.02
        }

        // Speeds in ticks per second: RPM * ticks per turn / 60 seconds.
        val target = rpm * RobotConstants.FLYWHEEL_TICKS_PER_REV / 60.0
        val error = target - motor1.velocity
        integral = (integral + error * dt)
        val power = RobotConstants.FLYWHEEL_P * error +
            RobotConstants.FLYWHEEL_I * integral +
            RobotConstants.FLYWHEEL_D * (error - lastError) / dt +
            RobotConstants.FLYWHEEL_F * target
        lastError = error

        val clipped = power.coerceIn(-1.0, 1.0)
        motor1.power = clipped
        motor2.power = clipped
    }

    /** How fast the flywheel is really spinning, in RPM. */
    fun currentRpm(): Double {
        return motor1.velocity * 60.0 / RobotConstants.FLYWHEEL_TICKS_PER_REV
    }

    /** True when the flywheel is on and close enough to its target speed to shoot. */
    fun isAtSpeed(): Boolean {
        if (targetRpm <= 0.0) return false
        return abs(currentRpm() - targetRpm) < RobotConstants.FLYWHEEL_RPM_TOLERANCE
    }

    fun stop() {
        setRpm(0.0)
    }
}
