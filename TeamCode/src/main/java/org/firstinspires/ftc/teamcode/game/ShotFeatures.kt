package org.firstinspires.ftc.teamcode.game

import com.pedropathing.math.Pose
import com.pedropathing.math.Velocity
import com.pedropathing.utils.Angle
import org.firstinspires.ftc.teamcode.RobotConstants
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

/**
 * The numbers that describe a shot, for the shot model (a small neural network that picks the
 * flywheel speed and hood angle). The logger and the model must compute them in exactly the same
 * way, so both use ShotFeatures.compute().
 *
 * "Turret" means the turret's center of rotation: the ball leaves from there, not from the robot center.
 */
data class ShotFeatures(
    /** Flat distance from the turret to the target, in inches. */
    val distanceInches: Double,
    /** How fast the turret moves toward the target, in inches per second (negative = moving away). */
    val towardSpeed: Double,
    /** How fast the turret moves to the left, as seen from the turret looking at the target (in/s). */
    val leftSpeed: Double,
    /** How fast the robot turns, in degrees per second (positive = counterclockwise). */
    val turnDegreesPerSecond: Double,
    /** How far around the side of the CELL we are, in degrees (0 = straight in front of its opening). */
    val offAxisDegrees: Double,
) {
    companion object {
        /**
         * robotPose and velocity come from odometry (velocity on the field: vx, vy in inches per second,
         * omega in radians per second). target is the CELL opening; cell says which way that opening faces.
         */
        fun compute(robotPose: Pose, velocity: Velocity, target: Pose, cell: HiveCell): ShotFeatures {
            val turret = turretPosition(robotPose)
            val turretVelocity = turretVelocity(robotPose, velocity)

            val dx = target.x() - turret.x()
            val dy = target.y() - turret.y()
            val distance = hypot(dx, dy)
            // A one-inch-long arrow pointing at the target, and one pointing to its left.
            val towardX = if (distance > 0.001) dx / distance else 1.0
            val towardY = if (distance > 0.001) dy / distance else 0.0
            val leftX = -towardY
            val leftY = towardX

            // The opening of an AUDIENCE_SIDE CELL faces the audience (-y), a FAR_SIDE one the far wall (+y).
            // We shoot toward the opening from in front of it, so "straight in" points the other way.
            val straightInHeading = if (cell == HiveCell.AUDIENCE_SIDE) Math.PI / 2 else -Math.PI / 2
            val shotHeading = Field.headingTo(turret, target)
            val offAxis = abs(Math.toDegrees(Angle.normalizeSigned(shotHeading - straightInHeading)))

            return ShotFeatures(
                distanceInches = distance,
                towardSpeed = turretVelocity.vx * towardX + turretVelocity.vy * towardY,
                leftSpeed = turretVelocity.vx * leftX + turretVelocity.vy * leftY,
                turnDegreesPerSecond = Math.toDegrees(velocity.omega),
                offAxisDegrees = offAxis,
            )
        }

        /** Where the turret's center of rotation is on the field. */
        fun turretPosition(robotPose: Pose): Pose {
            val h = robotPose.heading()
            val r = RobotConstants.TURRET_FORWARD_INCHES
            return Pose(robotPose.x() + r * cos(h), robotPose.y() + r * sin(h), h)
        }

        /**
         * How fast the turret moves across the field. While the robot turns, the turret (which is not at
         * the robot center) also swings around, so that is added to the robot's own velocity.
         */
        fun turretVelocity(robotPose: Pose, velocity: Velocity): Velocity {
            val h = robotPose.heading()
            val r = RobotConstants.TURRET_FORWARD_INCHES
            return Velocity(
                velocity.vx - velocity.omega * r * sin(h),
                velocity.vy + velocity.omega * r * cos(h),
                velocity.omega,
            )
        }

        /**
         * Where to aim so a ball fired while moving still hits the target. The ball keeps the turret's
         * velocity during its flight, so we aim that far "behind" the target: target - velocity x flight time.
         */
        fun leadPoint(target: Pose, robotPose: Pose, velocity: Velocity, flightSeconds: Double): Pose {
            val turretVelocity = turretVelocity(robotPose, velocity)
            return Pose(
                target.x() - turretVelocity.vx * flightSeconds,
                target.y() - turretVelocity.vy * flightSeconds,
                target.heading(),
            )
        }
    }
}
