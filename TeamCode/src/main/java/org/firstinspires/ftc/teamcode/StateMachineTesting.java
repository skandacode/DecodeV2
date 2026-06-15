package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.subsystems.Intakes;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

import java.util.Arrays;
import java.util.List;

import solverslib.gamepad.GamepadEx;
import solverslib.gamepad.GamepadKeys;

@Configurable
@TeleOp
public class StateMachineTesting extends LinearOpMode {
    Intakes intakes;
    Shooter shooter;
    Spindexer spindexer;
    Follower follower;
    public static double stallIntakeTime = 0.15;
    public static double openGateTime = 0.6;
    public static double intakeShooterVelo = 0.4;
    public static double hoodPos = 0.7;
    public static double shooterVelocity = 1800;
    public static Shooter.Goal targetGoal = Shooter.Goal.RED;
    public static double turretAngle = 0;

    public enum States{
        Intake,
        PreShoot,
        OpenUpperGate,
        Shoot,
    }



    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        follower = createFollower(hardwareMap);
        follower.setStartingPose(new Pose(60, 0, Math.toRadians(180)));
        intakes = new Intakes(hardwareMap);
        shooter = new Shooter(hardwareMap);
        spindexer = new Spindexer(hardwareMap);
        GamepadEx gamepadEx = new GamepadEx(gamepad1);
        GamepadKeys.Button shooterButton = GamepadKeys.Button.B;
        GamepadKeys.Button stopIntakeButton = GamepadKeys.Button.A;
        GamepadKeys.Button restartIntake = GamepadKeys.Button.Y;

        while (opModeInInit()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();
            telemetry.update();
        }

        waitForStart();
        follower.startTeleopDrive();
        System.out.println("started");

        StateMachine stateMachine = new StateMachineBuilder()
                .state(TeleopOnlyRapidFAR.States.Intake)
                .onEnter(() -> {
                    intakes.setGoodIntakePower(1);
                    shooter.setUpperGateOpen(false);
                    spindexer.setLowerGateOpen(true);
                    spindexer.setKickerPos(false);
                    spindexer.setPosition(Spindexer.SpindexerPosition.Shoot0);
                })
                .loop(()->{
                    if (gamepadEx.getButton(stopIntakeButton)){
                        intakes.setGoodIntakePower(0);
                    }
                    if (gamepadEx.getButton(restartIntake)){
                        intakes.setGoodIntakePower(1);
                    }
                })
                .transition(() -> gamepadEx.getButton(shooterButton), TeleopOnlyRapidFAR.States.OpenUpperGate)
                .state(TeleopOnlyRapidFAR.States.OpenUpperGate)
                .onEnter(() -> {
                    intakes.setGoodIntakePower(intakeShooterVelo);
                })
                .transitionTimed(stallIntakeTime, TeleopOnlyRapidFAR.States.PreShoot)
                .state(TeleopOnlyRapidFAR.States.PreShoot)
                .onEnter(() -> {
                    shooter.setUpperGateOpen(true);
                })
                .transitionTimed(0.1, TeleopOnlyRapidFAR.States.Shoot)
                .state(TeleopOnlyRapidFAR.States.Shoot)
                .onEnter(() -> {
                    spindexer.setKickerPos(true);
                })
                .transitionTimed(openGateTime, TeleopOnlyRapidFAR.States.Intake)
                .build();


        stateMachine.start();

        while (opModeIsActive()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();
            stateMachine.update();
            shooter.setTargetVelocity(shooterVelocity);
            shooter.setHood(hoodPos);
            shooter.setTurretPos(shooter.convertDegreestoServoPos(turretAngle));
            intakes.update();
            follower.update();
            shooter.update();
            spindexer.update();
            telemetry.addData("State: ", stateMachine.getState());
            telemetry.addData("Shooter Target", shooter.getTargetVelo());
            telemetry.addData("Shooter Velocity", shooter.getCurrentVelocity());
            telemetry.addData("angle distance", Arrays.toString(shooter.getAngleDistance(follower.getPose(), targetGoal)));
            telemetry.update();
        }
    }
}

