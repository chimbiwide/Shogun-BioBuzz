package org.firstinspires.ftc.teamcode

import com.bylazar.limelightproxy.LimelightProxyConfig
import com.bylazar.panels.PanelsConfig

/*
 * Turns Panels (the web dashboard) on or off. The switch is RobotConstants.USE_PANELS.
 *
 * Panels finds these two classes by itself when the robot app starts. Nothing else uses them.
 * Panels is NOT allowed at competitions (rule R704.D), so keep USE_PANELS = false for every event.
 *
 * Never run the "Enable/Disable Panels" OpMode (Panels adds it to the TeleOp list by itself):
 *  - With USE_PANELS = false, pressing START on it turns Panels ON until the robot is turned off and on
 *    again, and the "Panels is ON" INIT line cannot see that. If someone ran it, power-cycle the robot.
 *  - With USE_PANELS = true, it turns Panels off, and the robot app crashes once at the next power-on
 *    (a bug in this version of Panels, 1.0.3).
 *
 * After you change USE_PANELS from false to true and deploy, the robot app restarts once by itself
 * (the same Panels bug). That is expected: after that, Panels works.
 */
class PanelsSwitch : PanelsConfig() {
    init {
        isDisabled = !RobotConstants.USE_PANELS
    }
}

/** Panels also sends the Limelight's video through the robot's Wi-Fi, even when Panels is off. This stops it. */
class LimelightProxySwitch : LimelightProxyConfig() {
    init {
        isEnabled = RobotConstants.USE_PANELS
    }
}
