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
        GamepadKeys.Button relocalize = GamepadKeys.Button.LEFT_BUMPER;


        while (opModeInInit()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();
            telemetry.update();
        }

        waitForStart();
        follower.startTeleopDrive();
        System.out.println("started");

        StateMachine stateMachine = new StateMachineBuilder()
                .state(TeleopOnlyRapidManual.States.Intake)
                .onEnter(() -> {
                    intakes.setGoodIntakePower(1);
                    shooter.setUpperGateOpen(false);
                    spindexer.setLowerGateOpen(true);
                    spindexer.setKickerPos(false);
                    spindexer.setPosition(Spindexer.SpindexerPosition.Shoot0);
                })
                .loop(()->{
                    if ((intakes.getGoodBeamBreakOutside() && intakes.getGoodBeamBreakInside() && intakes.getGoodIntakeDetected())) {
                        intakes.setGoodIntakePower(0.4);
                    } else {
                        intakes.setFrontIntakePower(1);
                        intakes.setTransferIntakePower(0.6);
                    }
                })
                .transition(() -> gamepadEx.getButton(shooterButton), TeleopOnlyRapidManual.States.OpenUpperGate)
                .transition(() -> gamepadEx.getButton(stopIntakeButton), TeleopOnlyRapidManual.States.HoldBalls)

                .state(TeleopOnlyRapidManual.States.HoldBalls)
                .onEnter(() -> {
                })
                .loop(()->{
                    intakes.setGoodIntakePower(0.3);
                })
                .transition(() -> gamepadEx.getButton(shooterButton), TeleopOnlyRapidManual.States.OpenUpperGate)
                .transition(() -> gamepadEx.getButton(restartIntake), TeleopOnlyRapidManual.States.Intake)


                .state(TeleopOnlyRapidManual.States.OpenUpperGate)
                .onEnter(() -> {
                    intakes.setGoodIntakePower(0);
                    shooter.setUpperGateOpen(true);
                })
                .transitionTimed(0.1, TeleopOnlyRapidManual.States.Shoot)
                .state(TeleopOnlyRapidManual.States.Shoot)
                .onEnter(() -> {
                    intakes.setGoodIntakePower(1);
                    spindexer.setKickerPos(true);
                })
                .transitionTimed(0.35, TeleopOnlyRapidManual.States.Intake)
                .build();

        stateMachine.start();

        while (opModeIsActive()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();
            stateMachine.update();
            if (gamepadEx.getButton(relocalize)){
                follower.setPose(new Pose(-14.5, 56, Math.toRadians(90)));
            }
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

