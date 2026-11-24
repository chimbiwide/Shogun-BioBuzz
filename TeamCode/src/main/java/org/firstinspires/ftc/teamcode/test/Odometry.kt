package org.firstinspires.ftc.teamcode.test

import android.provider.SyncStateContract
import com.pedropathing.follower.Follower
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.pedro.Constants
import org.firstinspires.ftc.teamcode.teleOp.teleTemplate

@Autonomous(name = "Odometry Test")
class Odometry : OpMode() {

    private lateinit var follower: Follower
    override fun init() {
        follower = Constants.create(hardwareMap)
    }

    override fun loop() {
        // Only the localizer: follower.update() would also hold the wheels in brake mode, so pushing by hand is hard.
        follower.localizer.update()
        val pose = follower.pose()
        telemetry.addData(
            "Position",
            "x %.2f y %.2f heading %.2f".format(pose.x(), pose.y(), Math.toDegrees(pose.heading()))
        )
        telemetry.update()
    }

}