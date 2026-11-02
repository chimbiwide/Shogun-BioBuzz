package org.firstinspires.ftc.teamcode.auto

import com.pedropathing.math.Pose
import org.firstinspires.ftc.teamcode.RobotConstants
import org.firstinspires.ftc.teamcode.game.Alliance
import org.firstinspires.ftc.teamcode.game.Field
import org.firstinspires.ftc.teamcode.game.HiveCell

/**
 * Every place the robot drives to in autonomous, written for the RED alliance.
 * BLUE uses the same list: Field.forAlliance() turns each pose around the field center.
 *
 * x and y are inches and point at the CENTER of the robot. Headings are in degrees.
 * Look at game/Field.kt for how the field coordinates work.
 * Change these numbers on a practice field until the robot goes exactly where you want.
 *
 * Rules these poses follow:
 *  - Pedro turns the robot at the START of each path, and a turning robot needs about 12 inches
 *    of room on every side. So any pose that starts a turn is at least 12 inches from walls.
 *  - In autonomous we stay on our half of the field (x below 72), even while turning (rule G402).
 */
object AutoPoses {
    private const val FRONT = RobotConstants.CENTER_TO_FRONT_INCHES
    private const val BACK = RobotConstants.CENTER_TO_BACK_INCHES
    private const val SIDE = RobotConstants.CENTER_TO_SIDE_INCHES

    // ---------------- Starting spots ----------------
    // GARDEN routine and Simple autonomous. RED: back of the robot on the AUDIENCE wall, centered on
    // tile C1, intake facing the HIVE (BLUE is turned around: FAR wall, tile D6).
    val AUDIENCE_START = Field.pose(60.0, BACK, 90.0)
    // GO_AROUND routine. RED: intake against the FAR wall on tile C6, right side 2 inches from the
    // middle seam (BLUE: AUDIENCE wall, tile D1).
    // The robot drives backwards from here, so it does not have to turn.
    val FAR_START = Field.pose(72.0 - 2.0 - SIDE, Field.SIZE - FRONT, 90.0)

    // ---------------- Leaving the wall ----------------
    // Drive straight off the wall first, so there is room to turn.
    val LEAVE_AUDIENCE_WALL = Field.pose(56.0, 22.0, 90.0)
    val LEAVE_FAR_WALL = Field.pose(56.0, 118.0, 90.0)

    // ---------------- Our GARDEN ----------------
    // Our GARDEN's 4 POLLEN sit in a line along the audience wall, starting in the corner.
    // Line up above them with the intake facing the audience wall, then creep forward into them.
    val GARDEN_APPROACH = Field.pose(SIDE + 1.0, 24.0, -90.0)
    val GARDEN_PICKUP = Field.pose(SIDE + 1.0, FRONT + 1.0, -90.0)
    val GARDEN_BACK_OFF = Field.pose(20.0, 24.0, -90.0)   // back away from both walls before turning

    // ---------------- Shooting spots ----------------
    // A CELL's opening faces the wall that CELL points at, so we must shoot from that side of the HIVE.
    // In RED's view the CELL that starts up is on the audience side. After the HIVE tips, the far one is.
    val SHOOT_AUDIENCE_SIDE = facingOpening(52.0, 16.0, HiveCell.AUDIENCE_SIDE)
    // A little farther from the far wall than the audience spot: there is a FLOWER on the far wall.
    val SHOOT_FAR_SIDE = facingOpening(52.0, 124.0, HiveCell.FAR_SIDE)

    // ---------------- Going around the HIVE ----------------
    // A lane between the HIVE frame and our alliance wall. Drive along it to get to the other side.
    const val LANE_X = 28.0
    val LANE_AUDIENCE_SIDE = Field.pose(LANE_X, 40.0, 90.0)
    val LANE_FAR_SIDE = Field.pose(LANE_X, 104.0, 90.0)

    // ---------------- Parking ----------------
    // Two spots partly inside our LOADING ZONE (one for each robot of our alliance), not touching the wall.
    // We line up in the lane next to the spot, then turn and back straight in.
    val PARK_MIDDLE = Field.pose(12.5, 98.0, 0.0)         // the half of the LOADING ZONE nearer the middle
    val PARK_MIDDLE_LINE_UP = Field.pose(LANE_X, 98.0, 90.0)
    val PARK_CORNER = Field.pose(12.5, 121.0, 0.0)        // the half nearer the corner
    val PARK_CORNER_LINE_UP = Field.pose(LANE_X, 121.0, 90.0)

    /** A pose at (x, y) that faces the opening of one of RED's CELLS, so the turret only has to turn a little. */
    private fun facingOpening(x: Double, y: Double, cell: HiveCell): Pose {
        val spot = Pose(x, y, 0.0)
        val opening = Field.openingPosition(Alliance.RED, cell)
        return Pose(x, y, Field.headingTo(spot, opening))
    }
}
