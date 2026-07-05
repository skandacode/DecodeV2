package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.CachedMotor;

@TeleOp
@Configurable
public class MotorDirections extends LinearOpMode {
    CachedMotor frontLeft, frontRight, backLeft, backRight;
    CachedMotor frontIntake, backIntake, shooter1, shooter2;
    public static double frontLeftPower = 0.0;
    public static double frontRightPower = 0.0;
    public static double backLeftPower = 0.0;
    public static double backRightPower = 0.0;


    public static double frontIntakeMotorPower = 0.0;
    public static double backIntakeMotorPower = 0.0;
    public static double shooterPower1 = 0.0;
    public static double shooterPower2 = 0.0;



    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        frontLeft = new CachedMotor(hardwareMap, "frontleft");
        frontRight = new CachedMotor(hardwareMap, "frontright");
        backLeft = new CachedMotor(hardwareMap, "backleft");
        backRight = new CachedMotor(hardwareMap, "backright");

        frontIntake = new CachedMotor(hardwareMap, "goodIntakeMotor");
        backIntake = new CachedMotor(hardwareMap, "badIntakeMotor");

        shooter1 = new CachedMotor(hardwareMap, "outtakemotor1");
        shooter2 = new CachedMotor(hardwareMap, "outtakemotor2");


        waitForStart();

        while (opModeIsActive()) {
            frontLeft.set(frontLeftPower);
            frontRight.set(frontRightPower);
            backLeft.set(backLeftPower);
            backRight.set(backRightPower);

            frontIntake.set(frontIntakeMotorPower);
            backIntake.set(backIntakeMotorPower);

            shooter1.set(shooterPower1);
            shooter2.set(shooterPower2);


            telemetry.addData("Front left current", frontLeft.getCurrentDraw());
            telemetry.addData("Front right current", frontRight.getCurrentDraw());
            telemetry.addData("Back left current", backLeft.getCurrentDraw());
            telemetry.addData("Back right current", backRight.getCurrentDraw());


            telemetry.addData("front intake current", frontIntake.getCurrentDraw());
            telemetry.addData("back intake current", backIntake.getCurrentDraw());
            telemetry.addData("shooter 1 current", shooter1.getCurrentDraw());
            telemetry.addData("shooter2 current", shooter2.getCurrentDraw());


            frontLeft.update();
            frontRight.update();
            backLeft.update();
            backRight.update();
            frontIntake.update();
            backIntake.update();
            shooter1.update();
            shooter2.update();

            telemetry.update();
        }
    }
}
