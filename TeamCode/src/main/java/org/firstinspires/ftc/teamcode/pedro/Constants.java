package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
    c.frontLeftName.set("frontLeft");
    c.frontRightName.set("frontRight");
    c.backLeftName.set("backLeft");
    c.backRightName.set("backRight");
    c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
    c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
    c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
    c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
});

    // to be tuned
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
    c.name.set("pinpoint");
    c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
    c.xPodOffset.set(-2.774311275932732);
    c.yPodOffset.set(-3.612038319505106);
    c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    c.globalDistanceUnit.set(DistanceUnit.INCH);
    c.offsetUnits.set(DistanceUnit.INCH);
});

    // to be tuned
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {}
    );

    public static Follower create(HardwareMap hp) {
        return new Follower(
                new PinpointLocalizer(hp, localizerConfig),
                new Mecanum(hp, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
