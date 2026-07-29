package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.Pinpoint;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static MecanumConfig mecanumConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("frontleft");
                c.backLeftName.set("backleft");
                c.frontRightName.set("frontright");
                c.backRightName.set("backright");
                c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.manualBrakeMode.set(true);
            }
    );

    public static PinpointConfig pinpointConfig = new PinpointConfig(
            c -> {
                c.name.set("odo");
                c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
                c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
                c.xPodOffset.set(4.4274);
                c.yPodOffset.set(-3.18316);
                c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            }
    );

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                c.linearBrakeCoefficients.set(Matrix.diag(0.108277, 0.108277));
                c.quadraticBrakeCoefficients.set(Matrix.diag(.0013818584, .0013818584));

                Controller largeTranslationalForward = Controller.pid(0.081,0,0).plus(Controller.staticFeedforward(0.01));
                Controller smallTranslationalForward = Controller.pid(0.054,0,0).plus(Controller.staticFeedforward(0.0005));
                Controller smallTranslationalLateral = Controller.pid(.072,0,0).plus(Controller.staticFeedforward(0.0005));
                Controller largeTranslationalLateral = Controller.pid(0.108,0,0).plus(Controller.staticFeedforward(0.01));
                c.forwardTranslationalController.set(Controller.piecewise(Controller.zero).put(0.5, smallTranslationalForward).put(2.5, largeTranslationalForward));
                c.lateralTranslationalController.set(Controller.piecewise(Controller.zero).put(0.5, smallTranslationalLateral).put(2.5, largeTranslationalLateral));
                c.brakeController.set(Controller.pid(0.0285, 0, 0).plus(Controller.dynamicFeedforward(0.0085574)));
                c.brakeAccelFeedforward.set(Controller.dynamicFeedforward(0.00164713333));
                c.headingController.set(Controller.pid(1.5724,0,0.2183));
                c.maxBrakingPower.set(0.3);
                c.centripetalScaling.set(0.0005);
                c.headingDriveRatio.set(0.35);
                c.fullPowerCoast.set(true);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(new Pinpoint(h, pinpointConfig), new Mecanum(h, mecanumConfig), new Foresight(foresightConfig));
    }
//    public static FollowerConstants followerConstants = new FollowerConstants()
//            .mass(12.5)
//            .forwardZeroPowerAcceleration(-29.1544361869664)
//            .lateralZeroPowerAcceleration(-57.24011845264418)
//            .translationalPIDFCoefficients(new PIDFCoefficients(0.12, 0, 0.02, 0))
//            .headingPIDFCoefficients(new PIDFCoefficients(1.4426, 0, 0.2179, 0))
//            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.6412, 0 ,0.1141, 0))
//            .useSecondaryHeadingPIDF(true)
//            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.025,0.0,0.00008,0.6,0.01));
//    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);
//
//    public static Follower create(HardwareMap hardwareMap) {
//        return new FollowerBuilder(followerConstants, hardwareMap)
//                .pinpointLocalizer(localizerConstants)
//                .pathConstraints(pathConstraints)
//                .mecanumDrivetrain(driveConstants)
//                .build();
//    }
//    public static MecanumConstants driveConstants = new MecanumConstants()
//            .maxPower(1)
//            .rightFrontMotorName("frontright")
//            .rightRearMotorName("backright")
//            .leftRearMotorName("backleft")
//            .leftFrontMotorName("frontleft")
//            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
//            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
//            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
//            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
//            .xVelocity(81.2539893923782)
//            .yVelocity(63.61617048382135)
//            .useBrakeModeInTeleOp(true);
//    public static PinpointConstants localizerConstants = new PinpointConstants()
//            .forwardPodY(4.4274)
//            .strafePodX(-3.18316)
//            .distanceUnit(DistanceUnit.INCH)
//            .hardwareMapName("odo")
//            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
//            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
//            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
}
