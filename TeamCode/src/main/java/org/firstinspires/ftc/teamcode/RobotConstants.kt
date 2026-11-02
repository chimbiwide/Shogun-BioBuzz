package org.firstinspires.ftc.teamcode

import org.firstinspires.ftc.teamcode.tools.ShotSetting

/**
 * Almost every number you will want to tune lives in this file.
 * (Small step sizes and "close enough" tolerances sit next to the code that uses them.)
 *
 * Units are part of each name (INCHES, DEGREES, RPM, SECONDS).
 * Servo positions go from 0.0 to 1.0.
 * The drive motors and the Pinpoint are set up in pedro/Constants.java.
 */
object RobotConstants {

    // ---------------- OpMode names ----------------
    // Autonomous loads the matching teleop by its name, so the name is written only here.
    const val TELEOP_RED = "TeleOp Red"
    const val TELEOP_BLUE = "TeleOp Blue"

    // ---------------- Panels (web dashboard) ----------------
    // Panels is NOT allowed at competitions (rule R704.D). Keep this false for every event.
    // At home you may set it to true and deploy. See PanelsSwitch.kt.
    const val USE_PANELS = false

    // ---------------- Hardware names ----------------
    // These must match the names in the Robot Configuration on the Driver Station.
    const val INTAKE_MOTOR = "intake"
    const val TRANSFER_MOTOR = "transfer"
    const val FLYWHEEL_MOTOR = "flywheel"
    const val HOOD_SERVO = "hood"
    const val TURRET_SERVO_1 = "turret1"
    const val TURRET_SERVO_2 = "turret2"
    const val LIMELIGHT = "limelight"
    const val WEBCAM = "Webcam 1"

    // ---------------- Robot size ----------------
    // Measured from the CAD. Measure your real robot (turret pointing forward) and fix these!
    // They are used to place the robot when it starts against a wall, and to keep it off the walls.
    const val CENTER_TO_FRONT_INCHES = 8.6   // robot center to the front of the intake roller
    const val CENTER_TO_BACK_INCHES = 7.8    // robot center to the back (the flywheel motor sticks out a little)
    const val CENTER_TO_SIDE_INCHES = 7.5    // robot center to the end of the intake roller (the widest part)

    // ---------------- Driving ----------------
    const val SLOW_MODE_SPEED = 0.4          // drive speed while the slow-mode button is held
    const val AIM_TURN_GAIN = 1.0            // how hard the robot turns to face the HIVE (driver holds cross)
    // How hard the robot turns toward the closest POLLEN (driver holds left bumper):
    // turn power for each radian (57 degrees) the POLLEN is off to the side, but never more than POLLEN_MAX_TURN.
    const val POLLEN_TURN_GAIN = 1.0
    const val POLLEN_MAX_TURN = 0.5

    // ---------------- Intake and transfer ----------------
    const val INTAKE_POWER = 1.0
    const val EJECT_POWER = -0.8
    const val TRANSFER_FEED_POWER = 1.0      // pushes POLLEN into the flywheel
    const val TRANSFER_REVERSE_POWER = -0.6
    // Transfer speed while intaking. Keep 0.0: nothing holds POLLEN back, so any more and they reach the flywheel.
    const val TRANSFER_POWER_WHILE_INTAKING = 0.0

    // ---------------- Flywheel ----------------
    const val FLYWHEEL_REVERSED = true
    const val FLYWHEEL_TICKS_PER_REV = 28.0  // goBILDA and REV motors without a gearbox: 28
    const val FLYWHEEL_RPM_TOLERANCE = 100.0 // "ready to shoot" when this close to the target speed
    // Velocity PIDF for the flywheel motor. A good starting F is 32767 / (max ticks per second).
    // A 6000 RPM motor makes 6000 * 28 / 60 = 2800 ticks per second, so F = 32767 / 2800 = about 11.7.
    const val FLYWHEEL_P = 15.0
    const val FLYWHEEL_I = 0.0
    const val FLYWHEEL_D = 0.0
    const val FLYWHEEL_F = 11.7

    // ---------------- Hood ----------------
    // A small gear drives a big curved gear on the hood, so only part of the servo range is safe.
    // Find where the hood stops in both directions with the Shooter Tuner, then set these a little inside.
    const val HOOD_MIN_POSITION = 0.45
    const val HOOD_MAX_POSITION = 0.55
    // The hood's angle at those two positions, measured with a protractor (Shooter Tuner step 1).
    // Only used to log the hood angle for the shot model. Until you measure them (NaN = "not measured"),
    // the hood_deg column stays empty and the model can use hood_pos instead.
    const val HOOD_DEGREES_AT_MIN_POSITION = Double.NaN
    const val HOOD_DEGREES_AT_MAX_POSITION = Double.NaN

    // ---------------- Turret ----------------
    // Turret angle 0 = shooter points straight forward (over the intake).
    // Positive angles turn the turret to the LEFT (counterclockwise seen from above).
    // The CAD shows a small gear on a big ring gear (about 4 to 1), so the turret can probably turn
    // only 30 to 45 degrees each way. Use the "Shooter Tuner" OpMode to measure these numbers.
    const val TURRET_POSITION_AT_0_DEGREES = 0.5
    const val TURRET_CALIBRATION_DEGREES = 30.0      // a second angle the turret can reach (measure it)
    const val TURRET_POSITION_AT_CALIBRATION = 0.8   // the servo position at that angle
    const val TURRET_MIN_DEGREES = -30.0             // furthest the turret may turn right
    const val TURRET_MAX_DEGREES = 30.0              // furthest the turret may turn left
    // Both turret servos turn the same gear. If they fight (buzzing, and the turret does not move),
    // one of them turns the wrong way: flip this to true.
    const val TURRET_SERVO_2_REVERSED = false
    // How fast the turret turns. Step 2 in ShooterTuner.kt shows how to measure it.
    const val TURRET_DEGREES_PER_SECOND = 90.0
    // How far the turret's center of rotation is in front of the robot center (negative = behind it).
    const val TURRET_FORWARD_INCHES = -2.2

    // ---------------- Shooting ----------------
    // Distance from the turret to the opening of the CELL (inches) -> flywheel speed and hood position.
    // These are starting guesses: fill in real values with the "Shooter Tuner" OpMode.
    // Keep the rows sorted from closest to farthest, and the hood values inside the hood limits.
    val SHOT_TABLE = listOf(
        ShotSetting(24.0, 2800.0, 0.46),
        ShotSetting(36.0, 3000.0, 0.47),
        ShotSetting(48.0, 3200.0, 0.48),
        ShotSetting(60.0, 3400.0, 0.50),
        ShotSetting(72.0, 3650.0, 0.51),
        ShotSetting(96.0, 4100.0, 0.53),
        ShotSetting(120.0, 4600.0, 0.54),
    )
    const val MANUAL_DISTANCE_INCHES = 48.0   // teleop manual mode shoots as if the CELL were this far away
    // Autonomous: time spent feeding to shoot all 4 POLLEN. Time 4 shots on the robot and add 0.5 s,
    // so no POLLEN is left inside (picking up 4 more would make 5: rule G407).
    const val FEED_SECONDS = 2.0
    const val SPIN_UP_TIMEOUT_SECONDS = 2.5   // autonomous: shoot anyway if the flywheel is still not ready
    const val FEED_TIMEOUT_SECONDS = 5.0      // autonomous: stop trying to shoot after this long

    // ---------------- Shooting while moving ----------------
    // A POLLEN keeps the robot's speed when it leaves the robot. So when the robot moves, we aim at a
    // "lead" point: the target minus (turret velocity x flight time). Measure the flight time with a
    // slow-motion phone video (frames from the hood to the opening / frames per second).
    const val BALL_FLIGHT_SECONDS = 0.65
    const val USE_LEAD_IN_TELEOP = false      // the Shot Collector always leads; turn this on once it works there

    // ---------------- Webcam: AprilTags under the HIVE CELLS ----------------
    // The webcam looks up at the AprilTags under the HIVE CELLS. From them it can tell:
    //  - which of our CELLS points up (the tags under an up CELL are high in the air), and
    //  - where the robot is on the field (to fix odometry drift).
    const val USE_WEBCAM = true
    // Where the webcam lens sits on the robot. The webcam is not in the CAD: measure all of these!
    const val WEBCAM_FORWARD_INCHES = 5.4     // in front of the robot center (negative = behind it)
    const val WEBCAM_LEFT_INCHES = 3.0        // left of the robot center (negative = right of it)
    const val WEBCAM_HEIGHT_INCHES = 8.5      // lens height above the floor
    const val WEBCAM_PITCH_DEGREES = 40.0     // how far the camera is tilted up (0 = looking level)
    const val WEBCAM_YAW_DEGREES = 0.0        // 0 = looking forward, positive = turned to the left
    // A short exposure keeps pictures sharp while the robot moves; the gain brightens the dark picture.
    const val WEBCAM_EXPOSURE_MS = 6L
    const val WEBCAM_GAIN = 250
    // 1 = finds tags farthest away but slowest. 2 or 3 = faster, shorter range.
    const val WEBCAM_DECIMATION = 2.0f
    // Leave this false until the Shooter Tuner shows tag rows about 50 in high under the up CELL and
    // about 36 in under the down CELL. When true, autonomous follows what the webcam says about the
    // up CELL, and keeps its POLLEN if it sees none of our tags at a shooting spot.
    // (In teleop the webcam is only ever a hint on the screen: the operator decides.)
    const val USE_CAMERA_TO_FIND_UP_CELL = false
    // A tag row higher than this is under a CELL that points up (up: about 49.9 in, down: about 35.6 in).
    const val UP_CELL_MIN_TAG_HEIGHT_INCHES = 42.7
    // The camera must keep giving the same answer this long before we believe it (ignores flickers).
    const val CAMERA_CONFIRM_SECONDS = 0.4

    // ---------------- Fixing odometry with the webcam ----------------
    // Leave this false until the Shooter Tuner shows a "camera position" within about 1 inch of the real one.
    const val USE_TAGS_TO_FIX_POSITION = false
    const val TAG_FIX_GAIN = 0.3                  // each fix moves our position this part of the way (0 to 1)
    const val TAG_FIX_MAX_SPEED = 10.0            // inches per second: only fix while (almost) still
    const val TAG_FIX_MAX_TURN_DEGREES = 10.0     // degrees per second: turning blurs the picture and makes it wrong
    const val TAG_FIX_MAX_RANGE_INCHES = 100.0    // tags farther away are too small to trust
    const val TAG_FIX_MAX_JUMP_INCHES = 24.0      // a camera answer this far from odometry is a bad picture
    const val TAG_FIX_MIN_PERCENT = 50            // at least 2 of the 4 tags under a CELL must be seen

    // ---------------- Limelight: find POLLEN on the floor ----------------
    // The Limelight runs an object detection pipeline that finds yellow POLLEN: a color pipeline, or a
    // neural "detector" pipeline with a class called "pollen". Make it the Limelight's default pipeline.
    const val LIMELIGHT_PIPELINE = 0
    // Where the Limelight lens sits. In the CAD it is tilted 45 degrees UP and cannot see the floor:
    // re-aim it about 25 degrees DOWN, then measure all of these.
    const val LIMELIGHT_FORWARD_INCHES = 5.4
    const val LIMELIGHT_LEFT_INCHES = 0.0
    const val LIMELIGHT_HEIGHT_INCHES = 8.5
    const val LIMELIGHT_PITCH_DEGREES = -25.0     // negative = tilted down
    const val LIMELIGHT_YAW_DEGREES = 0.0
    const val POLLEN_MIN_CONFIDENCE = 0.5          // neural detector only: ignore guesses less sure than this (0 to 1)
    const val POLLEN_MAX_DISTANCE_INCHES = 72.0    // ignore POLLEN farther away than this

    // ---------------- Teleop ----------------
    const val TELEOP_SECONDS = 120.0          // how long the driver-controlled period lasts
    const val PARK_WARNING_SECONDS = 100.0    // buzz the driver this long into teleop (20 s left): go park

    // ---------------- Shot data logging (for training the shot model) ----------------
    // Files go to /sdcard/FIRST/data/shots on the Control Hub. See BIOBUZZ_GUIDE.md, section 10.
    const val SHOOTER_CONFIG_TAG = "v1"       // change it whenever the flywheel, its gearing or the hood changes
    const val LOG_SHOTS_IN_TELEOP = false     // practice only: also log teleop shots (label them on gamepad 2)
    const val SHOT_LOG_TRACE = false          // also write every loop to a "trace" file (to tune the shot detector)
    // A shot is found when the flywheel suddenly slows down while we feed (see logging/ShotDetector.kt).
    const val SHOT_DIP_RPM = 120.0            // how much it must slow down
    const val SHOT_DIP_WINDOW_SECONDS = 0.12  // ... compared with its fastest speed in this much time
    const val SHOT_FEED_GRACE_SECONDS = 0.4   // a ball can still be on its way this long after feeding stops
    const val SHOT_REARM_RPM = 40.0           // after a shot, speeding up again by this much allows the next one
    const val SHOT_MIN_GAP_SECONDS = 0.08     // two shots are at least this far apart
    const val SHOT_LABEL_TIMEOUT_SECONDS = 10.0  // teleop: a shot not labeled by then is saved as UNLABELED
    // Shot Collector: how long one feed may take before we give up on that ball.
    const val COLLECTOR_FEED_TIMEOUT_SECONDS = 1.0
    // Shot Collector, SWEEP mode: after every labeled shot, add one of these to the shot table's
    // speed and hood (picked at random), so the data shows where the shot goes in and where it misses.
    val SWEEP_RPM_OFFSETS = listOf(-150.0, -100.0, -50.0, 0.0, 50.0, 100.0, 150.0)
    val SWEEP_HOOD_OFFSETS = listOf(-0.01, 0.0, 0.01)

    // ---------------- Autonomous ----------------
    // Stop everything else and go park after this many seconds. Parking from the audience side
    // takes about 7 seconds, and autonomous is 30 seconds long.
    const val AUTO_PARK_START_SECONDS = 22.0
    const val AUTO_SLOW_SPEED = 0.4               // speed while driving into POLLEN (0 to 1)
    const val AUTO_PATH_TIMEOUT_SECONDS = 5.0     // stop waiting for a path that takes longer (robot stuck)
    const val AUTO_GARDEN_INTAKE_SECONDS = 1.5    // time spent pulling in the GARDEN's POLLEN
    const val AUTO_MAX_START_DELAY_SECONDS = 10.0 // the longest start delay the drivers can pick

    // ---------------- Simple autonomous (no Pedro) ----------------
    const val SIMPLE_AUTO_MAX_POWER = 0.5         // top drive power
    const val SIMPLE_AUTO_GAIN = 0.05             // drive power per inch still to go
    const val SIMPLE_AUTO_TURN_GAIN = 1.0         // turn power per radian still to turn
    // Below some power the wheels do not turn at all, and the robot would stop just short of each point.
    // Find the smallest power that still moves your robot (driving sideways too) and use a bit more.
    const val SIMPLE_AUTO_MIN_POWER = 0.12
    const val SIMPLE_AUTO_MIN_TURN_POWER = 0.08
}
