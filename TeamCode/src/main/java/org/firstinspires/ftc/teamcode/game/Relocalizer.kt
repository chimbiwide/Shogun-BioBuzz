package org.firstinspires.ftc.teamcode.game

import com.pedropathing.math.Pose
import org.firstinspires.ftc.teamcode.RobotConstants
import kotlin.math.abs
import kotlin.math.hypot

/** One row of 4 AprilTags under a HIVE CELL, as the webcam saw it in one picture. */
data class TagRowSighting(
    val alliance: Alliance,       // whose HIVE
    val cell: HiveCell,           // which CELL of that HIVE
    val seen: RobotPoint,         // the middle of the tag row, measured from the robot center
    val cellIsUp: Boolean,        // true if the row is high in the air: that CELL points up
    val percentFound: Int,        // how many of the 4 tags were in the picture: 25, 50, 75 or 100
    val pictureTimeNanos: Long,   // when the picture was taken (System.nanoTime clock)
)

/**
 * Fixes odometry drift with the webcam.
 *
 * We know where every tag row is on the field (Field.tagRowPosition). If the webcam sees a row
 * 30 inches ahead and 5 inches to the left, the robot must be 30 inches behind it and 5 to its right.
 * The heading always comes from the Pinpoint, which measures it very well.
 *
 * Pictures are only trusted while the robot is (almost) still: a moving robot gives blurry pictures,
 * and the picture is a little old by the time we get it.
 */
class Relocalizer {
    /** Where the camera thinks the robot is (from the last good picture), or null. Shown by the Shooter Tuner. */
    var cameraPosition: Pose? = null
        private set

    /** When we last moved the odometry position (System.nanoTime), or 0 if never. */
    var lastFixNanos = 0L
        private set

    private var ignorePicturesBeforeNanos = 0L

    /** Call this after setting the position by hand (corner reset): older pictures no longer match. */
    fun forgetOldPictures(nowNanos: Long) {
        ignorePicturesBeforeNanos = nowNanos
        cameraPosition = null
    }

    /**
     * Where the robot is, according to the tag rows the webcam sees right now (null if none is good).
     * Several rows in one picture are averaged.
     */
    fun positionFromTags(sightings: List<TagRowSighting>, robotHeading: Double, nowNanos: Long): Pose? {
        var sumX = 0.0
        var sumY = 0.0
        var count = 0
        for (sighting in sightings) {
            if (!isGoodSighting(sighting, nowNanos)) continue
            val row = Field.tagRowPosition(sighting.alliance, sighting.cell, sighting.cellIsUp)
            val estimate = CameraMath.robotPositionFrom(row.x(), row.y(), sighting.seen, robotHeading)
            sumX += estimate.x()
            sumY += estimate.y()
            count += 1
        }
        if (count == 0) return null
        val position = Pose(sumX / count, sumY / count, robotHeading)
        cameraPosition = position
        return position
    }

    /**
     * The corrected position to write into odometry this loop, or null to leave odometry alone.
     * Speed in inches per second, turn rate in degrees per second.
     */
    fun correctedPosition(
        sightings: List<TagRowSighting>,
        odometry: Pose,
        speed: Double,
        turnDegreesPerSecond: Double,
        nowNanos: Long,
    ): Pose? {
        // A picture taken while the robot was moving or turning shows where it was then, not where it is now.
        // So while it moves, also forget the pictures taken so far: they must never be used once it stops.
        if (speed > RobotConstants.TAG_FIX_MAX_SPEED || abs(turnDegreesPerSecond) > RobotConstants.TAG_FIX_MAX_TURN_DEGREES) {
            ignorePicturesBeforeNanos = nowNanos
            return null
        }
        val camera = positionFromTags(sightings, odometry.heading(), nowNanos) ?: return null
        // At most 10 fixes a second: each one is a message to the Pinpoint, which takes time.
        if (nowNanos - lastFixNanos < MIN_NANOS_BETWEEN_FIXES) return null
        // Far from odometry: more likely a bad picture than a real mistake. (The corner reset fixes big mistakes.)
        if (Field.distance(camera, odometry) > RobotConstants.TAG_FIX_MAX_JUMP_INCHES) return null

        lastFixNanos = nowNanos
        // Move only part of the way. One picture can be a little off; many pictures in a row average out.
        val gain = RobotConstants.TAG_FIX_GAIN
        val x = odometry.x() + gain * (camera.x() - odometry.x())
        val y = odometry.y() + gain * (camera.y() - odometry.y())
        return Pose(x, y, odometry.heading())
    }

    /** Seconds since odometry was last fixed by the camera (a big number if never). */
    fun secondsSinceFix(nowNanos: Long): Double {
        if (lastFixNanos == 0L) return 999.0
        return (nowNanos - lastFixNanos) / 1e9
    }

    private fun isGoodSighting(sighting: TagRowSighting, nowNanos: Long): Boolean {
        if (sighting.percentFound < RobotConstants.TAG_FIX_MIN_PERCENT) return false
        if (sighting.pictureTimeNanos < ignorePicturesBeforeNanos) return false
        if (nowNanos - sighting.pictureTimeNanos > MAX_PICTURE_AGE_NANOS) return false
        if (hypot(sighting.seen.forward, sighting.seen.left) > RobotConstants.TAG_FIX_MAX_RANGE_INCHES) return false
        // The row must be about as high as it should be. If not, the picture or the camera numbers are off.
        val expectedHeight = if (sighting.cellIsUp) Field.TAG_ROW_HEIGHT_UP else Field.TAG_ROW_HEIGHT_DOWN
        if (abs(sighting.seen.up - expectedHeight) > HEIGHT_TOLERANCE_INCHES) return false
        return true
    }

    companion object {
        private const val MAX_PICTURE_AGE_NANOS = 150_000_000L     // 0.15 s
        private const val MIN_NANOS_BETWEEN_FIXES = 100_000_000L   // 0.1 s
        private const val HEIGHT_TOLERANCE_INCHES = 5.0
    }
}
