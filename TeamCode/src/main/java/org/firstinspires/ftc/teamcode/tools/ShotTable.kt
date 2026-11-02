package org.firstinspires.ftc.teamcode.tools

import org.firstinspires.ftc.teamcode.RobotConstants

/** One row of the shot table: from this far away, spin the flywheel this fast and put the hood here. */
data class ShotSetting(val distanceInches: Double, val flywheelRpm: Double, val hoodPosition: Double)

/**
 * Finds the flywheel speed and hood position for any distance.
 *
 * Between two rows of the table it blends the two rows (linear interpolation).
 * Closer than the first row it uses the first row, farther than the last row it uses the last row.
 */
object ShotTable {
    fun lookUp(distanceInches: Double, table: List<ShotSetting> = RobotConstants.SHOT_TABLE): ShotSetting {
        val first = table.first()
        val last = table.last()
        if (distanceInches <= first.distanceInches) return first
        if (distanceInches >= last.distanceInches) return last

        for (i in 0 until table.size - 1) {
            val near = table[i]
            val far = table[i + 1]
            if (distanceInches <= far.distanceInches) {
                // 0.0 means "at the near row", 1.0 means "at the far row"
                val fraction = (distanceInches - near.distanceInches) / (far.distanceInches - near.distanceInches)
                val rpm = near.flywheelRpm + fraction * (far.flywheelRpm - near.flywheelRpm)
                val hood = near.hoodPosition + fraction * (far.hoodPosition - near.hoodPosition)
                return ShotSetting(distanceInches, rpm, hood)
            }
        }
        return last
    }
}
