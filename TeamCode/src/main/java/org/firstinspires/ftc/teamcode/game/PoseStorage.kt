package org.firstinspires.ftc.teamcode.game

import com.pedropathing.math.Pose

/**
 * Hands information from autonomous to teleop.
 *
 * A Kotlin "object" stays in memory between OpModes (until the robot app restarts),
 * so autonomous saves the robot's position here and teleop picks it up.
 * Teleop saves here too, so a teleop that had to be restarted carries on from where it was.
 */
object PoseStorage {
    var pose: Pose? = null
    var upCell: HiveCell? = null
    private var savedAtMillis = 0L

    fun save(robotPose: Pose, cell: HiveCell) {
        pose = robotPose
        upCell = cell
        savedAtMillis = System.currentTimeMillis()
    }

    /**
     * True if a position was saved less than a minute ago.
     * An older position is from a different match or practice run, and the robot has moved since.
     */
    fun isFresh(): Boolean {
        if (pose == null) return false
        return System.currentTimeMillis() - savedAtMillis < 60_000
    }
}
