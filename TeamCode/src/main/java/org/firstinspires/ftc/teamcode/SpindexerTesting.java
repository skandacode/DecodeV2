package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intakes;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.Tilt;


@Configurable
@TeleOp
public class SpindexerTesting extends LinearOpMode {
    Spindexer spindexer;

    public static double spindexerPosition = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        spindexer = new Spindexer(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {

            spindexer.setPosition(spindexerPosition);

            spindexer.update();
            telemetry.update();
        }
    }
}
