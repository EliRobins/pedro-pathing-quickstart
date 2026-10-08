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
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontRight");
        c.frontRightName.set("frontLeft");
        c.backLeftName.set("backRight");
        c.backRightName.set("backLeft");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(1.2827140327513689);
        c.yPodOffset.set(2.40526785062054);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.2175378228831978);
                Controller secondaryTranslationalForward = Controller.proportional(0.08037441283765978);
                Controller primaryTranslationalLateral = Controller.proportional(0.25659993934198455);
                Controller secondaryTranslationalLateral = Controller.proportional(0.09480682111020652);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.016603017331372937));
                c.brake.set(Controller.proportionalFeedforward(0.014112564731666995));

                c.headingFeedback.set(Controller.proportional(2.379269531600042));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.08024073949158272, 7.221439686120967E-4));

                c.linearBrakeCoefficients.set(Matrix.diag(0.06226843687054889, 0.09215626668181558));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0018086749831849316, 7.382121651256834E-4));

                c.maxAchievableForwardVelocity.set(61.027786354644185);
                c.maxAchievableStrafeVelocity.set(54.45585298292808);
                c.naturalForwardDeceleration.set(40.76508360694747);
                c.naturalStrafeDeceleration.set(50.388112516623316);
            }
    );
}