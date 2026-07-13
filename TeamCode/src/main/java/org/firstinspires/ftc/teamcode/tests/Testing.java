package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;


@Configurable
@TeleOp
public class Testing extends LinearOpMode {
    Intake intake;
    Kicker kicker;
    Shooter shooter;

    public static double goodIntakePower = 0.0;
    public static double badIntakePower = 0.0;

    public static double turretPosition = 0.5;
    public static double hoodPos = 0.5;

    public static int shooterTargetVelocity = 0;

    public static boolean kick = false;
    public static boolean lowerGateOpen = false;
    public static boolean upperGateOpen = false;

    public static boolean tilted = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        intake = new Intake(hardwareMap);
        kicker = new Kicker(hardwareMap);
        shooter = new Shooter(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            intake.setPower(goodIntakePower);
            shooter.setTargetVelocity(shooterTargetVelocity);
            kicker.setKicker(kick);
            shooter.setUpperGate(upperGateOpen);
            shooter.setHood(hoodPos);

            shooter.setTurretPos(turretPosition);

            intake.update();
            kicker.update();
            shooter.update();

            telemetry.addData("Shooter Velocity", shooter.getCurrentVelocity());
            telemetry.addData("Shooter Target Velocity", shooterTargetVelocity);

            telemetry.addData("Good intake distance", intake.getDistance());

            telemetry.addData("Good inside beam break", intake.getBeamBreakInside());
            telemetry.addData("Good outside beam break", intake.getBeamBreakOutside());



            telemetry.update();
        }
    }
}
