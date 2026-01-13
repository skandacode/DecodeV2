package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intakes;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;


@Configurable
@TeleOp
public class Testing extends LinearOpMode {
    Intakes intakes;
    Spindexer spindexer;
    Shooter shooter;

    public static double goodIntakePower = 0.0;
    public static double badIntakePower = 0.0;

    public static double spindexerPosition = 0.2;
    public static double turretPosition = 0.5;

    public static int shooterTargetVelocity = 0;

    public static boolean kick = false;
    public static boolean lowerGateOpen = false;
    public static boolean upperGateOpen = false;

    @Override
    public void runOpMode() throws InterruptedException {
        intakes = new Intakes(hardwareMap);
        spindexer = new Spindexer(hardwareMap);
        shooter = new Shooter(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            intakes.setGoodIntakePower(goodIntakePower);
            intakes.setBadIntakePower(badIntakePower);
            spindexer.setPosition(spindexerPosition);
            shooter.setTargetVelocity(shooterTargetVelocity);

            spindexer.setLowerGateOpen(lowerGateOpen);
            spindexer.setKickerPos(kick);
            shooter.setUpperGateOpen(upperGateOpen);

            shooter.setTurretPos(turretPosition);

            intakes.update();
            spindexer.update();
            shooter.update();

            telemetry.addData("Shooter Velocity", shooter.getCurrentVelocity());
            telemetry.update();
        }
    }
}
