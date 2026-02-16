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
public class Testing extends LinearOpMode {
    Intakes intakes;
    Spindexer spindexer;
    Shooter shooter;
    Tilt tilt;

    public static double goodIntakePower = 0.0;
    public static double badIntakePower = 0.0;

    public static Spindexer.SpindexerPosition spindexerPosition = Spindexer.SpindexerPosition.Shoot1;
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
        intakes = new Intakes(hardwareMap);
        spindexer = new Spindexer(hardwareMap);
        shooter = new Shooter(hardwareMap);
        tilt = new Tilt(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            intakes.setGoodIntakePower(goodIntakePower);
            intakes.setBadIntakePower(badIntakePower);
            spindexer.setPosition(spindexerPosition);
            shooter.setTargetVelocity(shooterTargetVelocity);

            spindexer.setLowerGateOpen(lowerGateOpen);
            spindexer.setKickerPos(kick);
            shooter.setUpperGateOpen(upperGateOpen);
            shooter.setHood(hoodPos);

            shooter.setTurretPos(turretPosition);

            if (tilted){
                tilt.tilt();
            }else{
                tilt.retract();
            }

            tilt.update();

            intakes.update();
            spindexer.update();
            shooter.update();

            telemetry.addData("Shooter Velocity", shooter.getCurrentVelocity());
            telemetry.addData("Shooter Target Velocity", shooterTargetVelocity);

            telemetry.addData("Bad intake distance", intakes.getBadIntakeDistance());
            telemetry.addData("Good intake distance", intakes.getGoodIntakeDistance());

            telemetry.addData("Bad beam break", intakes.getBadBeamBreak());
            telemetry.addData("Good inside beam break", intakes.getGoodBeamBreakInside());
            telemetry.addData("Good outside beam break", intakes.getGoodBeamBreakOutside());



            telemetry.update();
        }
    }
}
