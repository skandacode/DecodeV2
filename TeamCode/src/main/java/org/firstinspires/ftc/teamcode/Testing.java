package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Intakes;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;


@Configurable
public class Testing extends LinearOpMode {
    Intakes intakes;
    Spindexer spindexer;
    Shooter shooter;

    public static double goodIntakePower = 1.0;
    public static double badIntakePower = 1.0;

    public static double spindexerPosition = 0.2;

    public static int shooterTargetVelocity = 1500;

    @Override
    public void runOpMode() throws InterruptedException {
        intakes = new Intakes(hardwareMap);
        spindexer = new Spindexer(hardwareMap);
        shooter = new Shooter(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            intakes.setGoodIntakePower(goodIntakePower);
            intakes.setBadIntakeMotor(badIntakePower);
            spindexer.setPosition(spindexerPosition);
            shooter.setTargetVelocity(shooterTargetVelocity);

            intakes.update();
            spindexer.update();
            shooter.update();

            telemetry.addData("Shooter Velocity", shooter.getCurrentVelocity());
            telemetry.update();
        }
    }
}
