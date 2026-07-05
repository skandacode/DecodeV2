package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

@TeleOp
@Configurable
public class IntakeChecker extends LinearOpMode {
    Intake intake;
    Transfer transfer;
    Shooter shooter;

    public static double intakePower = 0.0;
    public static double transferPower = 0.0;

    public static boolean kick = false;
    public static boolean lowerGateOpen = true;

    public static boolean useRawSpindexerPos = false;
    public static Transfer.SpindexerPosition spindexerPosition = Transfer.SpindexerPosition.Shoot1;
    public static double rawSpindexerPosition = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        shooter = new Shooter(hardwareMap);

        waitForStart();

        while (opModeIsActive()){
            transfer.setKicker(kick);
            if (useRawSpindexerPos){
                transfer.setPosition(rawSpindexerPosition);
            }else {
                transfer.setPosition(spindexerPosition);
            }
            shooter.setUpperGate(false);
            transfer.setLowerGate(lowerGateOpen);

            intake.setIntakePower(intakePower);
            intake.setTransferPower(transferPower);

            telemetry.addData("Good Intake Distance", intake.getDistance());

            telemetry.addData("Good inside intake Beam break", intake.getBeamBreakInside());
            telemetry.addData("Good outside Beam break", intake.getBeamBreakOutside());


            intake.update();
            transfer.update();
            shooter.update();

            telemetry.update();
        }
    }
}
