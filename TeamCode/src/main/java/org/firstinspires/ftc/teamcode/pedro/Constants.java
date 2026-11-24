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
    c.xPodOffset.set(4.98);  // forward pod: 4.98 in LEFT of center (left = +)
    c.yPodOffset.set(1.575); // strafe pod: 40 mm (1.575 in) toward the intake/front (front = +)
    c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    c.globalDistanceUnit.set(DistanceUnit.INCH);
    c.offsetUnits.set(DistanceUnit.INCH);
});

    // to be tuned
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.23701570961595672);
                Controller secondaryTranslationalForward = Controller.proportional(0.08757097152669528);
                Controller primaryTranslationalLateral = Controller.proportional(0.28357512785848155);
                Controller secondaryTranslationalLateral = Controller.proportional(0.10477343247674004);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.011318766665569482));
                c.brake.set(Controller.proportionalFeedforward(0.009620951665734059));

                c.headingFeedback.set(Controller.proportional(3.7163641402313723));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.0652535869473493, 0.005766711265440045));

                c.linearBrakeCoefficients.set(Matrix.diag(0.0993831985963346, 0.05326188938891411));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.001485140573852914, 0.0020641810069002018));

                c.maxAchievableForwardVelocity.set(67.6927780535113);
                c.maxAchievableStrafeVelocity.set(62.304913180662695);
                c.naturalForwardDeceleration.set(32.88449936135031);
                c.naturalStrafeDeceleration.set(52.855107809053244);
            }
    );

    public static Follower create(HardwareMap hp) {
        Foresight foresight;
        try {
            foresight = new Foresight(foresightConfig);
        } catch (IllegalStateException e) {
            throw new IllegalStateException("Pedro is not tuned yet: run the Foresight Tuner "
                    + "(http://192.168.43.1:10158) and paste its results into Constants.foresightConfig.", e);
        }
        return new Follower(
                new PinpointLocalizer(hp, localizerConfig),
                new Mecanum(hp, drivetrainConfig),
                foresight
        );
    }
}
