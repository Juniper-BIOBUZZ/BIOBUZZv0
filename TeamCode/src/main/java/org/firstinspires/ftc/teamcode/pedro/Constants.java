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

public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("FLM");
                c.backLeftName.set("BLM");
                c.frontRightName.set("FRM");
                c.backRightName.set("BRM");

                c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);

                c.manualBrakeMode.set(true);
            }
    );

    public static PinpointConfig localizerConfig = new PinpointConfig(
            c -> {
                c.name.set("odo");
                c.xPodOffset.set(-5.79);
                c.yPodOffset.set(1.53);
                c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
                c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
            }
    );
//yes
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(1.0);
                Controller secondaryTranslationalForward = Controller.proportional(1.0);
                Controller primaryTranslationalLateral = Controller.proportional(1.0);
                Controller secondaryTranslationalLateral = Controller.proportional(1.0);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(1.0, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(1.0, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(1.0));
                c.brake.set(Controller.proportionalFeedforward(1.0));

                c.headingFeedback.set(Controller.proportional(1.0));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(1.0, 1.0));

                c.linearBrakeCoefficients.set(Matrix.diag(1.0,1.0));
                c.quadraticBrakeCoefficients.set(Matrix.diag(1.0,1.0));

                c.maxAchievableForwardVelocity.set(1.0);
                c.maxAchievableStrafeVelocity.set(1.0);
                c.naturalForwardDeceleration.set(1.0);
                c.naturalStrafeDeceleration.set(1.0);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}