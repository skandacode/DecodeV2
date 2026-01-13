package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import solverslib.hardware.motors.Motor;

@TeleOp
@Configurable
@Disabled
public class MotorDirections extends LinearOpMode {
    Motor frontLeft, frontRight, backLeft, backRight;
    public static double frontLeftPower = 0.0;
    public static double frontRightPower = 0.0;
    public static double backLeftPower = 0.0;
    public static double backRightPower = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = new Motor(hardwareMap, "frontleft");
        frontRight = new Motor(hardwareMap, "frontright");
        backLeft = new Motor(hardwareMap, "backleft");
        backRight = new Motor(hardwareMap, "backright");

        waitForStart();

        while (opModeIsActive()) {
            frontLeft.set(frontLeftPower);
            frontRight.set(frontRightPower);
            backLeft.set(backLeftPower);
            backRight.set(backRightPower);

            frontLeft.update();
            frontRight.update();
            backLeft.update();
            backRight.update();
        }
    }
}
