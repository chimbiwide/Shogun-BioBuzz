package org.firstinspires.ftc.teamcode.tools

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.util.ElapsedTime

/**
 * Reads the battery voltage. Each reading is a separate message to the hub and takes a few
 * milliseconds, so we read it at most 4 times a second and remember the value in between.
 */
class Battery(hardwareMap: HardwareMap) {
    private val sensor: VoltageSensor? = hardwareMap.voltageSensor.firstOrNull()
    private val timer = ElapsedTime()

    /** The last reading, in volts (NaN if there is no voltage sensor). */
    var volts = sensor?.voltage ?: Double.NaN
        private set

    /** Call every loop. */
    fun update() {
        if (timer.seconds() > 0.25) {
            volts = sensor?.voltage ?: Double.NaN
            timer.reset()
        }
    }
}
