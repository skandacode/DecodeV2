package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Kicker;


@Configurable
@TeleOp
public class SpindexerTesting extends LinearOpMode {
    Kicker kicker;

    public static double spindexerPosition = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        kicker = new Kicker(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {
            kicker.update();
            telemetry.update();
        }
    }
}
