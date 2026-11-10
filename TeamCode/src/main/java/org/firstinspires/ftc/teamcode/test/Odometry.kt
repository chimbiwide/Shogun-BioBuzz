package org.firstinspires.ftc.teamcode.test

import android.provider.SyncStateContract
import com.pedropathing.revhub.localizers.PinpointLocalizer
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.pedro.Constants
import org.firstinspires.ftc.teamcode.teleOp.teleTemplate

@Autonomous(name = "Odometry Test")
class Odometry : OpMode() {

    private lateinit var localizer: PinpointLocalizer
    override fun init() {
        localizer = PinpointLocalizer(hardwareMap, Constants.localizerConfig)
        localizer.reset()
    }

    override fun loop() {
        localizer.update()
        val pose = localizer.pose()
        telemetry.addData(
            "Position",
            "x %.2f y %.2f heading %.2f".format(pose.x(), pose.y(), Math.toDegrees(pose.heading()))
        )
        telemetry.update()
    }

}