package org.firstinspires.ftc.teamcode.tools

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.RobotConstants
import kotlin.math.abs

/**
 * The turntable that points the shooter. Two servos turn the same gear, so both always get the same position.
 *
 * Angles are in degrees: 0 = shooter points straight forward, positive = turned to the LEFT.
 */
class Turret(hardwareMap: HardwareMap) {
    private val servo1: Servo = hardwareMap.get(Servo::class.java, RobotConstants.TURRET_SERVO_1)
    private val servo2: Servo = hardwareMap.get(Servo::class.java, RobotConstants.TURRET_SERVO_2)
    private val timeSinceLastUpdate = ElapsedTime()

    /** The angle we last asked for (after the limits). */
    var angleDegrees = 0.0
        private set

    /** Where we think the turret is right now. Servos take time to turn, so this catches up at the turret's top speed. */
    var estimatedAngleDegrees = 0.0
        private set

    /** The servo position we last sent. */
    var servoPosition = RobotConstants.TURRET_POSITION_AT_0_DEGREES
        private set

    init {
        // Only sets the direction. Nothing may move during INIT.
        if (RobotConstants.TURRET_SERVO_2_REVERSED) {
            servo2.direction = Servo.Direction.REVERSE
        }
    }

    /** True if the turret can point this way without going past its limits. */
    fun canReach(degrees: Double): Boolean {
        if (degrees < RobotConstants.TURRET_MIN_DEGREES || degrees > RobotConstants.TURRET_MAX_DEGREES) {
            return false
        }
        // The calibration numbers might ask for a servo position the servo does not have.
        val position = angleToPosition(degrees)
        return position >= 0.0 && position <= 1.0
    }

    /** Turn the turret to an angle. Angles past the limits are stopped at the limit. Call this every loop. */
    fun setAngle(degrees: Double) {
        angleDegrees = degrees.coerceIn(RobotConstants.TURRET_MIN_DEGREES, RobotConstants.TURRET_MAX_DEGREES)
        sendToServos(angleToPosition(angleDegrees))

        // Move our estimate toward the new angle, but no faster than the turret can really turn.
        val mostItCouldTurn = RobotConstants.TURRET_DEGREES_PER_SECOND * timeSinceLastUpdate.seconds()
        timeSinceLastUpdate.reset()
        val stillToTurn = angleDegrees - estimatedAngleDegrees
        estimatedAngleDegrees += stillToTurn.coerceIn(-mostItCouldTurn, mostItCouldTurn)
    }

    /** True once the turret has (probably) reached the angle we asked for. */
    fun isSettled(): Boolean {
        return abs(angleDegrees - estimatedAngleDegrees) < 2.0
    }

    /** Send a raw servo position (0 to 1). Only the Shooter Tuner uses this, to find the calibration numbers. */
    fun setServoPosition(position: Double) {
        sendToServos(position)
    }

    private fun sendToServos(position: Double) {
        servoPosition = position.coerceIn(0.0, 1.0)
        servo1.position = servoPosition
        servo2.position = servoPosition
    }

    /**
     * Turns an angle into a servo position using the two calibration points in RobotConstants:
     * the position at 0 degrees and the position at TURRET_CALIBRATION_DEGREES.
     */
    private fun angleToPosition(degrees: Double): Double {
        val positionPerDegree = (RobotConstants.TURRET_POSITION_AT_CALIBRATION -
            RobotConstants.TURRET_POSITION_AT_0_DEGREES) / RobotConstants.TURRET_CALIBRATION_DEGREES
        return RobotConstants.TURRET_POSITION_AT_0_DEGREES + degrees * positionPerDegree
    }
}
