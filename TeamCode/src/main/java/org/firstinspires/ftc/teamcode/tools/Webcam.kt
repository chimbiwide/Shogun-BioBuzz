package org.firstinspires.ftc.teamcode.tools

import android.util.Size
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.RobotConstants
import org.firstinspires.ftc.teamcode.game.Alliance
import org.firstinspires.ftc.teamcode.game.CameraMath
import org.firstinspires.ftc.teamcode.game.CameraMount
import org.firstinspires.ftc.teamcode.game.Field
import org.firstinspires.ftc.teamcode.game.HiveCell
import org.firstinspires.ftc.teamcode.game.TagRowSighting
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import java.util.concurrent.TimeUnit

/**
 * The webcam looks at the AprilTags under the HIVE CELLS.
 *
 * Each CELL has a row of 4 tags. The FTC software finds the row as one group and tells us where the
 * group is, compared with the camera. From that we work out where the middle of the tag row is,
 * measured from the robot center (a TagRowSighting). That tells us:
 *  - which CELL points up: its tag row is high in the air (findUpCell), and
 *  - where the robot is on the field (Relocalizer.kt).
 *
 * You can watch what the webcam sees with "Camera Stream" on the Driver Station during INIT.
 * The robot app closes the camera by itself when the OpMode ends.
 */
class Webcam(hardwareMap: HardwareMap) {
    private val mount = CameraMount(
        forward = RobotConstants.WEBCAM_FORWARD_INCHES,
        left = RobotConstants.WEBCAM_LEFT_INCHES,
        height = RobotConstants.WEBCAM_HEIGHT_INCHES,
        pitchDegrees = RobotConstants.WEBCAM_PITCH_DEGREES,
        yawDegrees = RobotConstants.WEBCAM_YAW_DEGREES,
    )

    // Build a new tag finder in every OpMode (the FTC software does not allow reusing one).
    private val tagFinder: AprilTagProcessor = AprilTagProcessor.Builder()
        .setTagLibrary(AprilTagGameDatabase.getBioBuzzTagLibrary())
        .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
        .build()

    // null if there is no webcam in the robot configuration (the robot still works without it).
    private var portal: VisionPortal? = null
    private var cameraSettingsDone = false
    private var settingsProblem = ""
    private var lastPictureNanos = 0L

    /** The tag rows seen in the newest picture. Kept until a newer picture arrives. */
    var latestSightings: List<TagRowSighting> = emptyList()
        private set

    /** The tag rows from a picture that arrived since the last update() (empty if there is none). Use these to fix odometry. */
    var newSightings: List<TagRowSighting> = emptyList()
        private set

    init {
        tagFinder.setDecimation(RobotConstants.WEBCAM_DECIMATION)
        val camera = hardwareMap.tryGet(WebcamName::class.java, RobotConstants.WEBCAM)
        if (camera != null) {
            portal = VisionPortal.Builder()
                .setCamera(camera)
                // 640x480 has a built-in lens calibration for common webcams; without one, distances are wrong.
                .setCameraResolution(Size(640, 480))
                // Always choose the stream format (the FTC software can crash later if it is left empty).
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                // The Control Hub has no screen. "Camera Stream" on the Driver Station still works.
                .enableLiveView(false)
                .addProcessor(tagFinder)
                .build()
        }
    }

    /** True once the camera has started sending pictures (false if it is missing, unplugged or still starting up). */
    fun isStreaming(): Boolean = portal?.cameraState == VisionPortal.CameraState.STREAMING

    /** True if a picture arrived in the last half second (false if the webcam stopped or was unplugged). */
    fun isWorking(): Boolean = isStreaming() && System.nanoTime() - lastPictureNanos < 500_000_000L

    /** A few words for the screen. */
    fun status(): String {
        if (portal == null) return "not in the configuration (add \"Webcam 1\", or set USE_WEBCAM = false)"
        if (!isStreaming()) return "NOT STREAMING yet (wait a moment, or check the cable)"
        if (!isWorking()) return "NO PICTURES ARRIVING (check the cable)"
        if (settingsProblem.isNotEmpty()) return "working, but $settingsProblem"
        if (!cameraSettingsDone) return "working (setting the exposure...)"
        return "working, %.0f pictures per second".format(portal?.fps ?: 0f)
    }

    /**
     * Sets a short exposure, so the tags stay sharp while the robot moves. The camera must be streaming
     * first, so call this every loop: it does the work once, as soon as it can, and then does nothing.
     */
    fun applyCameraSettings() {
        val camera = portal ?: return
        if (cameraSettingsDone || !isStreaming()) return
        val exposure = camera.getCameraControl(ExposureControl::class.java)
        if (!exposure.isModeSupported(ExposureControl.Mode.Manual)) {
            settingsProblem = "this webcam has no manual exposure (pictures may blur while moving)"
            cameraSettingsDone = true
            return
        }
        if (exposure.mode != ExposureControl.Mode.Manual) {
            if (!exposure.setMode(ExposureControl.Mode.Manual)) {
                settingsProblem = "could not switch to manual exposure (pictures may blur while moving)"
                cameraSettingsDone = true
            }
            return   // give the camera a moment to switch; the next call sets the numbers
        }
        val exposureOk = exposure.setExposure(RobotConstants.WEBCAM_EXPOSURE_MS, TimeUnit.MILLISECONDS)
        val gainOk = camera.getCameraControl(GainControl::class.java).setGain(RobotConstants.WEBCAM_GAIN)
        if (!exposureOk || !gainOk) {
            settingsProblem = "exposure or gain not accepted: check WEBCAM_EXPOSURE_MS and WEBCAM_GAIN"
        }
        cameraSettingsDone = true
    }

    /** Call once every loop: reads the newest picture's tags (if a new picture arrived). */
    fun update() {
        // freshDetections is null when no new picture was processed since the last call.
        val detections = tagFinder.freshDetections
        if (detections == null) {
            newSightings = emptyList()
            if (!isWorking()) {
                latestSightings = emptyList()   // the camera stopped: its last picture is too old to trust
            }
            return
        }
        lastPictureNanos = System.nanoTime()
        val sightings = ArrayList<TagRowSighting>()
        for (detection in detections) {
            if (detection !is AprilTagClusterDetection) continue   // BIOBUZZ only has tag groups
            val alliance = Field.tagGroupAlliance(detection.metadata.name) ?: continue
            val cell = Field.tagGroupCell(detection.metadata.name) ?: continue

            // The software tells us where the group's origin is (the CELL opening) and how the group is turned (R).
            // The middle of the tag row is 7.19 in along the group's own y and -5.62 in along its z from the origin.
            // R turns those two numbers into the camera's directions (right, down, out of the lens).
            val pose = detection.rawPose
            val r = pose.R
            val right = r.get(0, 1) * TAG_ROW_Y + r.get(0, 2) * TAG_ROW_Z + pose.x
            val down = r.get(1, 1) * TAG_ROW_Y + r.get(1, 2) * TAG_ROW_Z + pose.y
            val outOfLens = r.get(2, 1) * TAG_ROW_Y + r.get(2, 2) * TAG_ROW_Z + pose.z
            val seen = CameraMath.cameraToRobot(right, down, outOfLens, mount)

            sightings.add(
                TagRowSighting(
                    alliance = alliance,
                    cell = cell,
                    seen = seen,
                    cellIsUp = seen.up > RobotConstants.UP_CELL_MIN_TAG_HEIGHT_INCHES,
                    percentFound = detection.percentClusterFound,
                    pictureTimeNanos = detection.frameAcquisitionNanoTime,
                )
            )
        }
        latestSightings = sightings
        newSightings = sightings
    }

    /** True if the newest picture shows any tags under our HIVE. */
    fun seesOurTags(alliance: Alliance): Boolean {
        return latestSightings.any { it.alliance == alliance }
    }

    /**
     * Which of our CELLS points up, from the newest picture (null if it shows none of our tags).
     * If both of our tag rows are in view, the higher one is under the up CELL.
     */
    fun findUpCell(alliance: Alliance): HiveCell? {
        val ours = latestSightings.filter { it.alliance == alliance }
        if (ours.isEmpty()) return null
        if (ours.size >= 2) {
            val highest = ours.maxByOrNull { it.seen.up } ?: return null
            return highest.cell
        }
        val only = ours[0]
        if (only.cellIsUp) return only.cell
        return only.cell.other()
    }

    companion object {
        // Where the middle of the 4-tag row is in the group's own directions (from the FTC tag library).
        private const val TAG_ROW_Y = 7.1874
        private const val TAG_ROW_Z = -5.622
    }
}
