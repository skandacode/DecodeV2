package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intakes;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

@TeleOp
@Configurable
public class IntakesVelocityChecker extends LinearOpMode {
    Intakes intakes;
    Spindexer spindexer;
    Shooter shooter;

    public static double goodPower = 0.0;
    public static double badPower = 0.0;
    public static boolean kick = false;
    public static boolean lowerGateOpen = true;
    public static Spindexer.SpindexerPosition spindexerPosition = Spindexer.SpindexerPosition.Shoot1;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        intakes = new Intakes(hardwareMap);
        spindexer = new Spindexer(hardwareMap);
        shooter = new Shooter(hardwareMap);

        waitForStart();

        while (opModeIsActive()){
            spindexer.setKickerPos(kick);
            spindexer.setPosition(spindexerPosition);
            shooter.setUpperGateOpen(false);
            spindexer.setLowerGateOpen(lowerGateOpen);

            intakes.setGoodIntakePower(goodPower);
            intakes.setBadIntakePower(badPower);

            telemetry.addData("Bad Intake Distance", intakes.getBadIntakeDistance());
            telemetry.addData("Good Intake Distance", intakes.getGoodIntakeDistance());

            telemetry.addData("Bad intake Beam break", intakes.getBadBeamBreak());
            telemetry.addData("Good inside intake Beam break", intakes.getGoodBeamBreakInside());
            telemetry.addData("Good outside Beam break", intakes.getGoodBeamBreakOutside());


            intakes.update();
            spindexer.update();
            shooter.update();

            telemetry.update();
        }
    }
}
