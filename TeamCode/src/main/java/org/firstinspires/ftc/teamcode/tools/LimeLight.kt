package org.firstinspires.ftc.teamcode.tools

import com.pedropathing.math.Pose
import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.RobotConstants
import org.firstinspires.ftc.teamcode.game.CameraMath
import org.firstinspires.ftc.teamcode.game.CameraMount
import org.firstinspires.ftc.teamcode.game.Field
import org.firstinspires.ftc.teamcode.game.RobotPoint
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.hypot
import kotlin.math.tan

/** One POLLEN the Limelight sees. */
data class PollenSighting(
    val seen: RobotPoint,          // where it is, measured from the robot center
    val fieldPosition: Pose,       // where it is on the field
    val distanceInches: Double,    // from the front of the intake
    val bearingDegrees: Double,    // how far the robot must turn to face it (positive = to the left)
)

/**
 * The Limelight runs object detection: it finds yellow POLLEN on the floor in front of the robot.
 * The pipeline can be a color pipeline (yellow blobs) or a neural "detector" with a class named "pollen".
 * The Limelight gives the angle to each ball. Balls lie on the floor, so the angle tells us where it is.
 */
class LimeLight(hardwareMap: HardwareMap, private val pipeline: Int = RobotConstants.LIMELIGHT_PIPELINE) {
    // tryGet gives null (instead of crashing) if there is no Limelight in the robot configuration.
    // Then the functions below give no answer and the robot still works without it.
    val limelight: Limelight3A? = hardwareMap.tryGet(Limelight3A::class.java, RobotConstants.LIMELIGHT)

    private val mount = CameraMount(
        forward = RobotConstants.LIMELIGHT_FORWARD_INCHES,
        left = RobotConstants.LIMELIGHT_LEFT_INCHES,
        height = RobotConstants.LIMELIGHT_HEIGHT_INCHES,
        pitchDegrees = RobotConstants.LIMELIGHT_PITCH_DEGREES,
        yawDegrees = RobotConstants.LIMELIGHT_YAW_DEGREES,
    )

    // The ball we chose last time, so we keep driving to the same one.
    private var target: PollenSighting? = null
    private val timeSinceTargetSeen = ElapsedTime()

    init {
        // pipelineSwitch() waits for the Limelight to answer, so it is only called here, in INIT.
        // Best: make the POLLEN pipeline the Limelight's default pipeline (in its web page), too.
        limelight?.pipelineSwitch(pipeline)
        limelight?.start()
    }

    /** True if the Limelight answered in the last 1/4 second (false if missing, unplugged or still starting up). */
    fun isConnected(): Boolean = limelight?.isConnected == true
    fun latestResult(): LLResult? = limelight?.latestResult
    fun stop() {
        limelight?.stop()
    }

    /** The pipeline the Limelight is really running (null if it is not answering). Show it during INIT to check it. */
    fun pipelineIndex(): Int? {
        if (!isConnected()) return null
        return latestResult()?.pipelineIndex
    }

    /** All the POLLEN in the newest picture. Empty if there are none, or the picture is old or from another pipeline. */
    fun pollenInView(robotPose: Pose): List<PollenSighting> {
        val result = latestResult() ?: return emptyList()
        if (!isConnected() || !result.isValid) return emptyList()
        if (result.staleness > 100) return emptyList()   // older than 0.1 second
        if (result.pipelineIndex != pipeline) return emptyList()   // still running another pipeline

        val balls = ArrayList<PollenSighting>()
        val detections = result.detectorResults
        if (detections.isNotEmpty()) {
            // Neural detector pipeline: keep only "pollen" guesses the network is sure enough about.
            for (detection in detections) {
                if (!detection.className.equals("pollen", ignoreCase = true)) continue
                // Some Limelight versions give confidence from 0 to 1, others from 0 to 100.
                val confidence = if (detection.confidence > 1.0) detection.confidence / 100.0 else detection.confidence
                if (confidence < RobotConstants.POLLEN_MIN_CONFIDENCE) continue
                addBall(
                    detection.targetXDegreesNoCrosshair, detection.targetYDegreesNoCrosshair,
                    detection.targetXDegrees, detection.targetYDegrees, robotPose, balls
                )
            }
        } else {
            // Color pipeline: every yellow blob that passed the Limelight's filters.
            for (blob in result.colorResults) {
                addBall(
                    blob.targetXDegreesNoCrosshair, blob.targetYDegreesNoCrosshair,
                    blob.targetXDegrees, blob.targetYDegrees, robotPose, balls
                )
            }
        }
        return balls
    }

    /**
     * The POLLEN to go for: the nearest one (a ball off to the side counts as a little farther, because
     * the robot must turn first). Keeps choosing the same ball while it is still seen, so the robot
     * does not jump back and forth between two balls. Null if there is none.
     */
    fun closestPollen(robotPose: Pose): PollenSighting? {
        val balls = pollenInView(robotPose)
        if (balls.isEmpty()) {
            if (timeSinceTargetSeen.seconds() > 0.25) target = null
            return null
        }
        val previous = target
        var choice: PollenSighting? = null
        if (previous != null) {
            // The same ball as before: within 6 inches of where it was.
            choice = balls.firstOrNull { Field.distance(it.fieldPosition, previous.fieldPosition) < 6.0 }
        }
        if (choice == null) {
            choice = balls.minByOrNull { it.distanceInches + 0.25 * abs(it.bearingDegrees) }
        }
        target = choice
        timeSinceTargetSeen.reset()
        return choice
    }

    /**
     * Turns the angles to one ball into its position and adds it to the list (if it is on the floor and
     * not too far). The "NoCrosshair" angles are measured from the true center of the picture, which is
     * what the math needs. Some pipelines do not send them (then both are exactly 0): then we use the
     * normal angles (keep the crosshair in the middle of the picture).
     */
    private fun addBall(
        txNoCrosshair: Double, tyNoCrosshair: Double, tx: Double, ty: Double,
        robotPose: Pose, balls: ArrayList<PollenSighting>,
    ) {
        val sent = !(txNoCrosshair == 0.0 && tyNoCrosshair == 0.0)
        val right = if (sent) txNoCrosshair else tx
        val up = if (sent) tyNoCrosshair else ty
        val seen = CameraMath.floorPoint(right, up, Field.POLLEN_RADIUS_INCHES, mount) ?: return
        val distance = hypot(seen.forward - RobotConstants.CENTER_TO_FRONT_INCHES, seen.left)
        if (distance > RobotConstants.POLLEN_MAX_DISTANCE_INCHES) return
        val bearing = Math.toDegrees(atan2(seen.left, seen.forward))
        balls.add(PollenSighting(seen, CameraMath.toField(seen, robotPose), distance, bearing))
    }

    companion object {
        fun getDistance(a1: Double, a2: Double, h1: Double, h2: Double): Double {
            val angToGoal = a1 + a2
            val angleRadians = angToGoal * (PI / 180.0)
            return (h2 - h1) / tan(angleRadians)
        }
    }
}
