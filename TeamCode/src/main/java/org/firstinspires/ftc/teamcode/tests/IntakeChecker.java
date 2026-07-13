package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;

@TeleOp
@Configurable
public class IntakeChecker extends LinearOpMode {
    Intake intake;
    Kicker kicker;
    Shooter shooter;

    public static double intakePower = 0.0;
    public static double transferPower = 0.0;

    public static boolean kick = false;
    public static boolean lowerGateOpen = true;

    public static boolean useRawSpindexerPos = false;
    public static double rawSpindexerPosition = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        intake = new Intake(hardwareMap);
        kicker = new Kicker(hardwareMap);
        shooter = new Shooter(hardwareMap);

        waitForStart();

        while (opModeIsActive()){
            kicker.setKicker(kick);
            shooter.setUpperGate(false);

            intake.setIntakePower(intakePower);
            intake.setTransferPower(transferPower);

            telemetry.addData("Good Intake Distance", intake.getDistance());

            telemetry.addData("Good inside intake Beam break", intake.getBeamBreakInside());
            telemetry.addData("Good outside Beam break", intake.getBeamBreakOutside());


            intake.update();
            kicker.update();
            shooter.update();

            telemetry.update();
        }
    }
}
