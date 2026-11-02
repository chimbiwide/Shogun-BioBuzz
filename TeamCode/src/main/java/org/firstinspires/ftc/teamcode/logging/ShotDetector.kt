package org.firstinspires.ftc.teamcode.logging

import org.firstinspires.ftc.teamcode.RobotConstants

/**
 * Finds the moment each POLLEN leaves the robot, without any extra sensor.
 *
 * When the flywheel throws a ball, it suddenly slows down (a "dip"), then speeds up again.
 * A shot is counted when ALL of these are true:
 *  - the flywheel is on,
 *  - we are feeding, or stopped feeding less than SHOT_FEED_GRACE_SECONDS ago
 *    (the ball may still be on its way, and teleop stops feeding as soon as the speed drops),
 *  - in the last SHOT_DIP_WINDOW_SECONDS the speed fell at least SHOT_DIP_RPM more than the target did,
 *    and it is now at least half of SHOT_DIP_RPM below the target (a robot driving toward the CELL lowers
 *    the target a little every loop: that is not a dip),
 *  - the target was not lowered in one step in the last half second (slowing down on purpose looks like a dip),
 *  - the last shot was at least SHOT_MIN_GAP_SECONDS ago.
 * After a shot, the next one can only be counted once the flywheel has sped up again by SHOT_REARM_RPM.
 */
class ShotDetector {
    /** One loop's flywheel numbers. */
    private class Sample(val seconds: Double, val rpm: Double, val targetRpm: Double)

    /** Time of the last shot: the loop just before the flywheel started slowing down (seconds). */
    var launchSeconds = 0.0
        private set

    /** Flywheel speed at that moment: the speed the ball really got (RPM). */
    var launchRpm = 0.0
        private set

    /** How much the flywheel slowed down during the last shot (RPM). Grows until the dip is over. */
    var dipRpm = 0.0
        private set

    /** True while the flywheel is still slowed down after a shot. */
    var inDip = false
        private set

    private val recent = ArrayDeque<Sample>()   // the last SHOT_DIP_WINDOW_SECONDS of loops
    private var lowestRpm = 0.0
    private var lastFeedSeconds = -1000.0
    private var lastShotSeconds = -1000.0
    private var previousTargetRpm = 0.0
    private var lastTargetStepDownSeconds = -1000.0

    /**
     * Call this every loop with the time (seconds), whether we are feeding POLLEN into the flywheel
     * this loop, the target speed and the measured speed. Returns true on the loop a new shot is found.
     */
    fun update(seconds: Double, feeding: Boolean, targetRpm: Double, rpm: Double): Boolean {
        if (feeding) {
            lastFeedSeconds = seconds
        }
        // The target dropped a lot in one loop (someone lowered it): the flywheel brakes on purpose for a moment.
        if (previousTargetRpm - targetRpm > 50.0) {
            lastTargetStepDownSeconds = seconds
        }
        previousTargetRpm = targetRpm

        recent.addLast(Sample(seconds, rpm, targetRpm))
        while (recent.first().seconds < seconds - RobotConstants.SHOT_DIP_WINDOW_SECONDS) {
            recent.removeFirst()
        }

        if (inDip) {
            // Follow the dip down to its lowest point, then wait until the flywheel speeds up again.
            if (rpm < lowestRpm) {
                lowestRpm = rpm
                dipRpm = launchRpm - lowestRpm
            }
            val spedUpAgain = rpm >= lowestRpm + RobotConstants.SHOT_REARM_RPM
            if (spedUpAgain && seconds - lastShotSeconds >= RobotConstants.SHOT_MIN_GAP_SECONDS) {
                inDip = false
                // Forget the old speeds, so the next dip is measured from where the flywheel is now.
                recent.clear()
                recent.addLast(Sample(seconds, rpm, targetRpm))
            }
            return false
        }

        // The fastest loop in the window, and how much the speed fell since then (minus what the target fell).
        // (">=" so that if several loops were equally fast, we take the latest one.)
        var fastest = recent.first()
        for (sample in recent) {
            if (sample.rpm >= fastest.rpm) fastest = sample
        }
        val targetFell = maxOf(0.0, fastest.targetRpm - targetRpm)

        val flywheelOn = targetRpm > 0.0
        val fedRecently = seconds - lastFeedSeconds <= RobotConstants.SHOT_FEED_GRACE_SECONDS
        val bigDrop = (fastest.rpm - rpm) - targetFell >= RobotConstants.SHOT_DIP_RPM
        // A ball pulls the flywheel well BELOW its target. Slowing down to a new, lower target stays near it.
        val belowTarget = rpm < targetRpm - RobotConstants.SHOT_DIP_RPM / 2
        val targetWasLowered = seconds - lastTargetStepDownSeconds < 0.5
        val longEnoughSinceLastShot = seconds - lastShotSeconds >= RobotConstants.SHOT_MIN_GAP_SECONDS

        if (flywheelOn && fedRecently && bigDrop && belowTarget && !targetWasLowered && longEnoughSinceLastShot) {
            // The launch moment: the LAST loop where the flywheel was still about as far above its target as
            // it got in the window (30 rpm leaves room for the speed reading's small jumps).
            var highestAboveTarget = -1e9
            for (sample in recent) {
                highestAboveTarget = maxOf(highestAboveTarget, sample.rpm - sample.targetRpm)
            }
            var launch = fastest
            for (sample in recent) {
                if (sample.rpm - sample.targetRpm >= highestAboveTarget - 30.0) launch = sample
            }

            inDip = true
            launchSeconds = launch.seconds
            launchRpm = launch.rpm
            lowestRpm = rpm
            dipRpm = launchRpm - rpm
            lastShotSeconds = seconds
            return true
        }
        return false
    }
}
