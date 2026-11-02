package org.firstinspires.ftc.teamcode.game

import com.pedropathing.math.Pose
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.tan

/** A point measured from the robot center: inches forward, inches to the left, and inches up from the floor. */
data class RobotPoint(val forward: Double, val left: Double, val up: Double)

/**
 * Where a camera sits on the robot: its lens position (forward, left and height in inches),
 * how far it is tilted up (pitch, degrees, negative = tilted down) and turned left (yaw, degrees).
 * The camera must not be rolled (tilted sideways).
 */
data class CameraMount(
    val forward: Double,
    val left: Double,
    val height: Double,
    val pitchDegrees: Double,
    val yawDegrees: Double,
)

/**
 * Math that turns what a camera sees into positions on the robot and on the field.
 * Every function here is plain math, so it can be tested on a computer.
 */
object CameraMath {

    /**
     * The FTC AprilTag software gives positions in the camera's own directions: right, down
     * (in the picture) and straight out of the lens. This turns such a point into a point
     * measured from the robot center.
     */
    fun cameraToRobot(right: Double, down: Double, outOfLens: Double, mount: CameraMount): RobotPoint {
        // 1. Undo the tilt. A camera tilted up sees "straight out of the lens" partly upward.
        val pitch = Math.toRadians(mount.pitchDegrees)
        val forwardFlat = outOfLens * cos(pitch) + down * sin(pitch)
        val upFromLens = outOfLens * sin(pitch) - down * cos(pitch)
        // 2. Undo the turn (yaw), then add where the lens sits on the robot.
        return levelCameraToRobot(forwardFlat, -right, upFromLens, mount)
    }

    /**
     * The Limelight gives the angles to a target: tx (degrees, + = right of the picture's center) and
     * ty (degrees, + = above the center). A ball lies on the floor, so its center is objectHeight inches
     * up: that is enough to work out where it is.
     * Returns null if that direction never comes down to the ball's height (the target is above the horizon).
     */
    fun floorPoint(txDegrees: Double, tyDegrees: Double, objectHeight: Double, mount: CameraMount): RobotPoint? {
        // The direction from the lens to the target, in the camera's directions (right, up, out of the lens) is
        // (tan tx, tan ty, 1). Undo the tilt to get forward / left / up directions on a level robot.
        val pitch = Math.toRadians(mount.pitchDegrees)
        val tanX = tan(Math.toRadians(txDegrees))
        val tanY = tan(Math.toRadians(tyDegrees))
        val rayForward = cos(pitch) - tanY * sin(pitch)
        val rayLeft = -tanX
        val rayUp = sin(pitch) + tanY * cos(pitch)
        if (rayUp > -0.02) {
            return null   // pointing level or up (or only 1 degree down): it never reaches the floor
        }
        // Follow that direction down until it is objectHeight above the floor.
        val steps = (mount.height - objectHeight) / -rayUp
        return levelCameraToRobot(steps * rayForward, steps * rayLeft, objectHeight - mount.height, mount)
    }

    /**
     * We know where a point is on the field (fieldX, fieldY) and where the robot sees it (seen).
     * The robot's heading (radians) comes from odometry. This gives the position of the robot center.
     */
    fun robotPositionFrom(fieldX: Double, fieldY: Double, seen: RobotPoint, robotHeading: Double): Pose {
        // Turn "forward / left of the robot" into "x / y on the field", then step back from the point.
        val dx = seen.forward * cos(robotHeading) - seen.left * sin(robotHeading)
        val dy = seen.forward * sin(robotHeading) + seen.left * cos(robotHeading)
        return Pose(fieldX - dx, fieldY - dy, robotHeading)
    }

    /** Where a point seen by the robot is on the field (the heading stays the robot's heading). */
    fun toField(point: RobotPoint, robotPose: Pose): Pose {
        val h = robotPose.heading()
        val x = robotPose.x() + point.forward * cos(h) - point.left * sin(h)
        val y = robotPose.y() + point.forward * sin(h) + point.left * cos(h)
        return Pose(x, y, h)
    }

    /** A point measured from a level camera (forward, left, up from the lens) -> measured from the robot center. */
    private fun levelCameraToRobot(forward: Double, left: Double, up: Double, mount: CameraMount): RobotPoint {
        val yaw = Math.toRadians(mount.yawDegrees)
        return RobotPoint(
            mount.forward + forward * cos(yaw) - left * sin(yaw),
            mount.left + forward * sin(yaw) + left * cos(yaw),
            mount.height + up,
        )
    }
}
