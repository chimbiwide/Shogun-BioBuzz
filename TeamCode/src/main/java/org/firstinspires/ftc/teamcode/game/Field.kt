package org.firstinspires.ftc.teamcode.game

import com.pedropathing.math.Pose
import org.firstinspires.ftc.teamcode.RobotConstants
import kotlin.math.PI
import kotlin.math.atan2
import kotlin.math.hypot

enum class Alliance { RED, BLUE }

/** Our HIVE has two CELLS. One is closer to the audience, one is closer to the far wall. */
enum class HiveCell {
    AUDIENCE_SIDE,
    FAR_SIDE;

    fun other(): HiveCell {
        if (this == AUDIENCE_SIDE) return FAR_SIDE
        return AUDIENCE_SIDE
    }
}

/**
 * A map of the BIOBUZZ field. All distances are in inches.
 *
 * Stand where the audience stands and look at the field:
 *  - (0, 0) is the near-left corner, where the RED alliance wall meets the audience wall.
 *  - x grows to the right, toward the BLUE alliance wall (0 to 144).
 *  - y grows away from you, toward the far wall (0 to 144).
 *  - Heading 0 degrees faces the BLUE wall, 90 degrees faces the far wall (counterclockwise is positive).
 *
 * Pedro Pathing uses radians for heading, so use pose(x, y, degrees) to make poses in degrees.
 */
object Field {
    const val SIZE = 144.0

    // The HIVE frame is in the middle of the field. Each HIVE swings on a pivot 43.95 inches above the floor,
    // tilting 30 degrees one way or the other.
    const val RED_HIVE_X = 59.25     // the two HIVES are 25.5 inches apart
    const val BLUE_HIVE_X = 84.75
    const val HIVE_Y = 72.0
    // A CELL's opening is at its OUTER end (the end away from the pivot), about this far from the pivot.
    // The opening faces the wall that CELL points at. Aim here, and tune the shot table to this point.
    const val OPENING_OFFSET = 16.5
    // You can only shoot into a CELL from in front of its opening: at least this far past it.
    const val OPEN_SIDE_MARGIN = 15.0

    // Each CELL has a row of 4 AprilTags on its bottom, facing down toward the floor (manual section 9.9).
    // The middle of that row is at these heights and distances from the pivot (along y):
    const val TAG_ROW_HEIGHT_UP = 49.87      // inches above the floor, when that CELL points up
    const val TAG_ROW_HEIGHT_DOWN = 35.60    // ... when that CELL points down
    const val TAG_ROW_OFFSET_UP = 13.06      // inches from the pivot, when that CELL points up
    const val TAG_ROW_OFFSET_DOWN = 11.66    // ... when that CELL points down

    // POLLEN are balls about 2.8 inches across, so their center is this high above the floor.
    const val POLLEN_RADIUS_INCHES = 1.4

    /** Makes a Pedro Pose from inches and degrees. */
    fun pose(x: Double, y: Double, headingDegrees: Double): Pose {
        return Pose(x, y, Math.toRadians(headingDegrees))
    }

    /**
     * The BLUE half of the field is the RED half spun halfway around the center of the field.
     * We write every autonomous pose for RED, and this turns it into the matching BLUE pose.
     * Spinning halfway around twice gets you back where you started, so this also turns a BLUE
     * pose into the matching RED pose.
     */
    fun forAlliance(redPose: Pose, alliance: Alliance): Pose {
        if (alliance == Alliance.RED) return redPose
        return Pose(SIZE - redPose.x(), SIZE - redPose.y(), redPose.heading() + PI)
    }

    /** Where the opening of a CELL is on the field (seen from above). This is what we aim at. */
    fun openingPosition(alliance: Alliance, cell: HiveCell): Pose {
        val x = if (alliance == Alliance.RED) RED_HIVE_X else BLUE_HIVE_X
        val y = if (cell == HiveCell.AUDIENCE_SIDE) HIVE_Y - OPENING_OFFSET else HIVE_Y + OPENING_OFFSET
        return Pose(x, y, 0.0)
    }

    /**
     * True if the robot is in front of a CELL's opening, so a shot can go in.
     * An AUDIENCE_SIDE cell opens toward the audience, so the robot must be between it and the audience
     * wall. A FAR_SIDE cell opens toward the far wall. Behind a CELL you would only hit its back.
     */
    fun canShootFrom(robotPose: Pose, opening: Pose): Boolean {
        if (opening.y() < HIVE_Y) {
            return robotPose.y() < opening.y() - OPEN_SIDE_MARGIN
        }
        return robotPose.y() > opening.y() + OPEN_SIDE_MARGIN
    }

    /** At the start of a match, the CELL that points at a FLOWER is tilted down, so the other one is up. */
    fun startingUpCell(alliance: Alliance): HiveCell {
        if (alliance == Alliance.RED) return HiveCell.AUDIENCE_SIDE
        return HiveCell.FAR_SIDE
    }

    /**
     * Where the middle of a CELL's tag row is on the field (seen from above), when that CELL points up
     * or down. The webcam measures where it sees this point, which tells us where the robot is.
     */
    fun tagRowPosition(alliance: Alliance, cell: HiveCell, cellIsUp: Boolean): Pose {
        val x = if (alliance == Alliance.RED) RED_HIVE_X else BLUE_HIVE_X
        val offset = if (cellIsUp) TAG_ROW_OFFSET_UP else TAG_ROW_OFFSET_DOWN
        val y = if (cell == HiveCell.AUDIENCE_SIDE) HIVE_Y - offset else HIVE_Y + offset
        return Pose(x, y, 0.0)
    }

    /**
     * The FTC software names the 4 tags under each CELL as one group: "RED AUDIENCE", "RED SCORING",
     * "BLUE AUDIENCE" or "BLUE SCORING" (tags 34-37, 30-33, 38-41, 42-45).
     * This gives the alliance of that group's HIVE, or null for any other name.
     */
    fun tagGroupAlliance(groupName: String): Alliance? {
        if (groupName.startsWith("RED")) return Alliance.RED
        if (groupName.startsWith("BLUE")) return Alliance.BLUE
        return null
    }

    /** Which CELL a tag group is under. "SCORING" means the scoring-table side, which is our FAR_SIDE. */
    fun tagGroupCell(groupName: String): HiveCell? {
        if (groupName.endsWith("AUDIENCE")) return HiveCell.AUDIENCE_SIDE
        if (groupName.endsWith("SCORING")) return HiveCell.FAR_SIDE
        return null
    }

    /**
     * A spot that is easy to find by feel: the corner on the drivers' LEFT along their own alliance wall.
     * The robot faces into the field, with its back on the alliance wall and its left side on the other wall.
     * Drive the robot into this corner and reset the position to fix odometry drift.
     */
    fun cornerPose(alliance: Alliance): Pose {
        val redCorner = pose(
            RobotConstants.CENTER_TO_BACK_INCHES,
            SIZE - RobotConstants.CENTER_TO_SIDE_INCHES,
            0.0
        )
        return forAlliance(redCorner, alliance)
    }

    /** Straight-line distance between two points on the field. */
    fun distance(from: Pose, to: Pose): Double {
        return hypot(to.x() - from.x(), to.y() - from.y())
    }

    /** The heading (radians) that points from one spot to another. */
    fun headingTo(from: Pose, to: Pose): Double {
        return atan2(to.y() - from.y(), to.x() - from.x())
    }
}
