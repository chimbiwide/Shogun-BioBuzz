package org.firstinspires.ftc.teamcode.tools

import com.pedropathing.math.Pose
import com.pedropathing.math.Velocity
import com.pedropathing.utils.Angle
import org.firstinspires.ftc.teamcode.game.Alliance
import org.firstinspires.ftc.teamcode.game.Field
import org.firstinspires.ftc.teamcode.game.HiveCell
import org.firstinspires.ftc.teamcode.game.ShotFeatures
import org.firstinspires.ftc.teamcode.logging.ShotLogger

/** Collects one loop's numbers for the ShotLogger, so the OpModes only need one line for it. */
object ShotLogData {

    /**
     * target = the CELL opening. aimPoint = where the turret aims (the same as target, or a lead point
     * when shooting on the move). flightSeconds = the flight time used for the lead (0 = no lead).
     */
    fun make(
        seconds: Double,
        loopMs: Double,
        alliance: Alliance,
        cell: HiveCell,
        mode: String,
        pose: Pose,
        velocity: Velocity,
        target: Pose,
        aimPoint: Pose,
        flightSeconds: Double,
        shooter: Shooter,
        manualMode: Boolean,
        feeding: Boolean,
        forced: Boolean,
        batteryVolts: Double,
        secondsSinceTagFix: Double,
    ): ShotLogger.LoopData {
        // How far the aim was moved off the target to lead a moving shot (degrees, + = to the left).
        val turret = ShotFeatures.turretPosition(pose)
        val leadDegrees = Math.toDegrees(
            Angle.normalizeSigned(Field.headingTo(turret, aimPoint) - Field.headingTo(turret, target))
        )
        return ShotLogger.LoopData(
            seconds = seconds,
            loopMs = loopMs,
            alliance = alliance.name,
            cell = cell.name,
            mode = mode,
            x = pose.x(),
            y = pose.y(),
            headingDegrees = Math.toDegrees(pose.heading()),
            vx = velocity.vx,
            vy = velocity.vy,
            targetX = target.x(),
            targetY = target.y(),
            features = ShotFeatures.compute(pose, velocity, target, cell),
            turretCommandDegrees = shooter.turret.angleDegrees,
            turretEstimateDegrees = shooter.turret.estimatedAngleDegrees,
            turretSettled = shooter.turret.isSettled(),
            leadDegrees = leadDegrees,
            flightSeconds = flightSeconds,
            aimDistanceInches = shooter.distance,
            distanceTrimInches = shooter.distanceTrimInches,
            turretTrimDegrees = shooter.turretTrimDegrees,
            manualMode = manualMode,
            tableRpm = shooter.setting.flywheelRpm,
            tableHood = shooter.setting.hoodPosition,
            rpmOffset = shooter.rpmOffset,
            hoodOffset = shooter.hoodOffset,
            targetRpm = shooter.flywheel.targetRpm,
            rpm = shooter.flywheel.currentRpm(),
            hoodPosition = shooter.hood.position,
            hoodDegrees = shooter.hood.angleDegrees(),
            ready = shooter.isReady(),
            forced = forced,
            feeding = feeding,
            batteryVolts = batteryVolts,
            secondsSinceTagFix = secondsSinceTagFix,
        )
    }
}
