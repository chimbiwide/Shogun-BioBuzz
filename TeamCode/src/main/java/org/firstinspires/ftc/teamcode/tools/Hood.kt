package org.firstinspires.ftc.teamcode.tools

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.RobotConstants

/** The adjustable hood changes how steep the POLLEN fly out of the shooter. */
class Hood(hardwareMap: HardwareMap) {
    private val servo: Servo = hardwareMap.get(Servo::class.java, RobotConstants.HOOD_SERVO)

    /** The servo position we last sent. */
    var position = 0.0
        private set

    fun setPosition(newPosition: Double) {
        // never go past the safe limits
        position = newPosition.coerceIn(RobotConstants.HOOD_MIN_POSITION, RobotConstants.HOOD_MAX_POSITION)
        servo.position = position
    }

    /** Send a raw servo position, ignoring the limits. Only the Shooter Tuner uses this, to find the limits. */
    fun setServoPosition(rawPosition: Double) {
        position = rawPosition.coerceIn(0.0, 1.0)
        servo.position = position
    }

    /**
     * The hood angle in degrees, worked out from the servo position with the two angles measured in
     * RobotConstants (a straight line between them). NaN ("not a number") until those are measured.
     */
    fun angleDegrees(): Double {
        val positionRange = RobotConstants.HOOD_MAX_POSITION - RobotConstants.HOOD_MIN_POSITION
        val fraction = (position - RobotConstants.HOOD_MIN_POSITION) / positionRange
        val angleRange = RobotConstants.HOOD_DEGREES_AT_MAX_POSITION - RobotConstants.HOOD_DEGREES_AT_MIN_POSITION
        return RobotConstants.HOOD_DEGREES_AT_MIN_POSITION + fraction * angleRange
    }
}
