package org.firstinspires.ftc.teamcode.tools

import com.pedropathing.math.Pose
import com.pedropathing.utils.Angle
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.RobotConstants
import org.firstinspires.ftc.teamcode.game.Field
import kotlin.math.cos
import kotlin.math.sin

/**
 * Everything that launches POLLEN: flywheel + hood + turret.
 *
 * Every loop, call aimAt(...) (or aimStraight(...) in manual mode). It points the turret,
 * sets the hood, and sets the flywheel speed for the distance to the target.
 */
class Shooter(hardwareMap: HardwareMap) {
    val flywheel = Flywheel(hardwareMap)
    val hood = Hood(hardwareMap)
    val turret = Turret(hardwareMap)

    /** The flywheel only spins when this is true. */
    var isOn = false

    // Small fixes the operator can make during a match.
    var distanceTrimInches = 0.0    // more = shoot as if the target were farther away
    var turretTrimDegrees = 0.0     // more = aim more to the left

    // Added on top of the shot table. Only the Shot Collector changes these (to try nearby settings).
    var rpmOffset = 0.0
    var hoodOffset = 0.0

    /** Distance from the turret to the target (inches), from the last aim. */
    var distance = 0.0
        private set

    /** Where the turret wants to point (degrees), before the turret limits. */
    var wantedTurretAngle = 0.0
        private set

    /** False when the robot is behind the target CELL: its opening faces the other way. */
    var onOpenSide = true
        private set

    /** Flywheel speed and hood position picked for the current distance. */
    var setting = ShotTable.lookUp(48.0)
        private set

    /**
     * Aim at a CELL's opening from where the robot is now. Call this every loop.
     * When shooting on the move, target can be a "lead" point next to the opening: then pass the real
     * opening too, so the side check (WRONG SIDE) uses the real opening.
     */
    fun aimAt(robotPose: Pose, target: Pose, opening: Pose = target) {
        pointTurretAt(robotPose, target)
        onOpenSide = Field.canShootFrom(robotPose, opening)
        useDistance(distance)
    }

    /** Manual mode: point the turret straight ahead and shoot as if the target is this far away. */
    fun aimStraight(distanceInches: Double) {
        wantedTurretAngle = turretTrimDegrees
        turret.setAngle(wantedTurretAngle)
        distance = distanceInches
        onOpenSide = true   // in manual mode the drivers decide where to shoot from
        useDistance(distance)
    }

    /** Only points the turret at the target (and measures the distance). Does not touch the flywheel or hood. */
    fun pointTurretAt(robotPose: Pose, target: Pose) {
        val turretSpot = turretSpot(robotPose)
        // Angle to the target on the field, minus the way the robot faces = angle for the turret.
        // normalizeSigned keeps it between -180 and 180 degrees (so 350 degrees becomes -10).
        val angleOnField = Field.headingTo(turretSpot, target)
        val angleFromFront = Angle.normalizeSigned(angleOnField - robotPose.heading())
        wantedTurretAngle = Math.toDegrees(angleFromFront) + turretTrimDegrees
        turret.setAngle(wantedTurretAngle)
        distance = Field.distance(turretSpot, target)
    }

    /** Distance (inches) from the turret to the target. */
    fun distanceTo(robotPose: Pose, target: Pose): Double {
        return Field.distance(turretSpot(robotPose), target)
    }

    /** True if the target is outside the turret's limits (the driver needs to turn the robot). */
    fun targetOutOfReach(): Boolean {
        return !turret.canReach(wantedTurretAngle)
    }

    /** True when it is OK to feed POLLEN into the flywheel. */
    fun isReady(): Boolean {
        return isOn && onOpenSide && !targetOutOfReach() && flywheel.isAtSpeed() && turret.isSettled()
    }

    /** What the shooter is waiting for, in a few words. "READY" when isReady() is true. */
    fun status(): String {
        if (!isOn) return "FLYWHEEL OFF"
        if (!onOpenSide) return "WRONG SIDE: the CELL's opening faces the other way"
        if (targetOutOfReach()) return "TURN THE ROBOT: the turret cannot reach"
        if (!flywheel.isAtSpeed()) {
            return "SPINNING UP %.0f / %.0f rpm".format(flywheel.currentRpm(), flywheel.targetRpm)
        }
        if (!turret.isSettled()) return "TURRET MOVING"
        return "READY"
    }

    /** The turret is not at the robot center, so work out where the turret is on the field. */
    private fun turretSpot(robotPose: Pose): Pose {
        val heading = robotPose.heading()
        val x = robotPose.x() + RobotConstants.TURRET_FORWARD_INCHES * cos(heading)
        val y = robotPose.y() + RobotConstants.TURRET_FORWARD_INCHES * sin(heading)
        return Pose(x, y, heading)
    }

    private fun useDistance(distanceInches: Double) {
        setting = ShotTable.lookUp(distanceInches + distanceTrimInches)
        hood.setPosition(setting.hoodPosition + hoodOffset)
        if (isOn) {
            flywheel.setRpm(setting.flywheelRpm + rpmOffset)
        } else {
            flywheel.setRpm(0.0)
        }
    }
}
