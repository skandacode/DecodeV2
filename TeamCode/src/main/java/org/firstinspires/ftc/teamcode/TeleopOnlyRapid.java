package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.followerConstants;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.pedroPathing.PanelsDrawing;
import org.firstinspires.ftc.teamcode.pedroPathing.Tuning;
import org.firstinspires.ftc.teamcode.subsystems.Intakes;
import org.firstinspires.ftc.teamcode.subsystems.Position;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.Tilt;

import java.io.IOException;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;

import solverslib.gamepad.GamepadEx;
import solverslib.gamepad.GamepadKeys;
import solverslib.hardware.motors.Motor;

@Configurable
@TeleOp
public class TeleopOnlyRapid extends LinearOpMode {
    Intakes intakes;
    Spindexer spindexer;
    Shooter shooter;
    Follower follower;
    Tilt tilt;

    Motor frontLeft, frontRight, backLeft, backRight;
    Motor intake1, intake2, shooter1, shooter2;

    public static Shooter.Goal target = Shooter.Goal.BLUE;
    public static double powerOffsetIncrements = 20;
    public static double turretOffsetIncrements = 2;


    public Pose relocalizePos = new Pose(-14.5, -56, Math.toRadians(-90));

    public static boolean allianceBlue = true;
    public static boolean telemetryCurrent = true;

    public enum States{
        Intake,
        TransferOff,
        PulseOut,
        PulseIn,
        HoldBalls,
        OpenUpperGate,
        Shoot,
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        GamepadEx gamepadEx = new GamepadEx(gamepad1);

        intakes = new Intakes(hardwareMap);
        spindexer = new Spindexer(hardwareMap);
        shooter = new Shooter(hardwareMap);
        follower = createFollower(hardwareMap);
        tilt = new Tilt(hardwareMap);


        frontLeft = new Motor(hardwareMap, "frontleft");
        frontRight = new Motor(hardwareMap, "frontright");
        backLeft = new Motor(hardwareMap, "backleft");
        backRight = new Motor(hardwareMap, "backright");

        intake1 = new Motor(hardwareMap, "frontIntake");
        intake2 = new Motor(hardwareMap, "transferIntake");

        shooter1 = new Motor(hardwareMap, "shooterMotor1");
        shooter2 = new Motor(hardwareMap, "shooterMotor2");

        GamepadKeys.Button slowModeButton = GamepadKeys.Button.RIGHT_BUMPER;
        GamepadKeys.Button positionResetButton = GamepadKeys.Button.LEFT_BUMPER;
        GamepadKeys.Button shooterButton = GamepadKeys.Button.B;
        GamepadKeys.Button stopIntakeButton = GamepadKeys.Button.A;
        GamepadKeys.Button restartIntake = GamepadKeys.Button.Y;

        follower.setStartingPose(Position.pose);

        StateMachine stateMachine = new StateMachineBuilder()
                .state(States.Intake)
                .onEnter(()->{
                    intakes.setGoodIntakePower(1);
                    shooter.setUpperGateOpen(false);
                    spindexer.setLowerGateOpen(true);
                    spindexer.setKickerPos(false);
                    spindexer.setPosition(Spindexer.SpindexerPosition.Shoot0);
                })
                .transition(()->intakes.getGoodBeamBreakInside() && intakes.getGoodIntakeDetected(), States.TransferOff)
                .transition(()->gamepadEx.getButton(stopIntakeButton), States.HoldBalls)

                .transition(()->gamepadEx.getButton(shooterButton), States.OpenUpperGate)

                .state(States.TransferOff)
                .onEnter(()->intakes.setTransferIntakePower(0.3))
                .transition(()->intakes.getGoodBeamBreakOutside(), States.PulseOut)
                .transition(()->gamepadEx.getButton(shooterButton), States.OpenUpperGate)

                .state(States.PulseOut)
                .onEnter(()->intakes.setFrontIntakePower(-0.5))
                .transitionTimed(0.2)

                .state(States.PulseIn)
                .onEnter(()->intakes.setFrontIntakePower(1))
                .transitionTimed(1)

                .state(States.HoldBalls)
                .onEnter(()->intakes.setGoodIntakePower(0.1))
                .transition(()->gamepadEx.getButton(shooterButton), States.OpenUpperGate)
                .transition(()->gamepadEx.getButton(restartIntake), States.Intake)


                .state(States.OpenUpperGate)
                .onEnter(()->{
                    shooter.setUpperGateOpen(true);
                    intakes.setGoodIntakePower(1);
                })
                .transitionTimed(0.1, States.Shoot)
                .state(States.Shoot)
                .onEnter(()->{
                    spindexer.setKickerPos(true);
                })
                .transitionTimed(0.7, States.Intake)
                .build();

        while (opModeInInit()) {
            for (LynxModule hub : allHubs) hub.clearBulkCache();
            follower.update();
            if (gamepad1.a) {
                target = Shooter.Goal.BLUE;
                allianceBlue=true;
                relocalizePos = new Pose(-14.5, -56, Math.toRadians(-90));
            }
            if (gamepad1.b) {
                target = Shooter.Goal.RED;
                allianceBlue=false;
                relocalizePos = new Pose(-14.5, 56, Math.toRadians(90));
            }

            telemetry.addData("Shooter Target", target);
            telemetry.addData("Current Pos", follower.getPose());
            telemetry.update();
        }

        waitForStart();

        stateMachine.start();
        follower.startTeleopDrive();

        long lastLoopTime = System.nanoTime();
        while (opModeIsActive()) {
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
            gamepadEx.readButtons();
            follower.update();
            Position.pose = follower.getPose();
            telemetry.addData("Angle and distance:", Arrays.toString(shooter.getAngleDistance(Position.pose, target)));
            shooter.aimAtTarget(Position.pose, target);

            //shooter.aimAtTarget(Position.pose, new Pose(targetX, targetY));

            double forward = gamepadEx.getLeftY();
            double strafe = gamepadEx.getLeftX();
            double turn = gamepadEx.getRightX();
            if (gamepadEx.getButton(slowModeButton)) {
                forward *= 0.3;
                strafe *= 0.3;
                turn *= 0.3;
            }

            follower.setTeleOpDrive(forward, -1*strafe, -1*turn, true);

            if (gamepadEx.wasJustPressed(positionResetButton)){
                follower.setPose(relocalizePos);
                if (allianceBlue) {
                    Shooter.powerOffset = 0;
                    Shooter.turretOffset = 0;
                }else{
                    Shooter.powerOffset = 0;
                    Shooter.turretOffset = 2;
                }
            }
            if (gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
                Shooter.powerOffset -= powerOffsetIncrements;
            }
            if (gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)){
                Shooter.turretOffset -= turretOffsetIncrements;
            }
            if (gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)){
                Shooter.turretOffset += turretOffsetIncrements;
            }
            if (gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
                Shooter.powerOffset += powerOffsetIncrements;
            }
            if (gamepadEx.isDown(GamepadKeys.Button.TOUCHPAD)){
                tilt.tilt();
            }else{
                tilt.retract();
            }
            stateMachine.update();

            telemetry.addData("Current Pos", follower.getPose());
            telemetry.addData("Shooter Target", shooter.getTargetVelo());
            telemetry.addData("Shooter Velocity", shooter.getCurrentVelocity());
            telemetry.addData("Spindexer kick", spindexer.is_kick);
            telemetry.addData("Statemachine State", stateMachine.getState());


            if (telemetryCurrent) {
                double totalCurrent = 0;
                double frontLeftCurrent = frontLeft.getCurrentDraw();
                double frontRightCurrent = frontRight.getCurrentDraw();
                double backLeftCurrent = backLeft.getCurrentDraw();
                double backRightCurrent = backRight.getCurrentDraw();


                telemetry.addData("Front left current", frontLeftCurrent);
                telemetry.addData("Front right current", frontRightCurrent);
                telemetry.addData("Back left current", backLeftCurrent);
                telemetry.addData("Back right current", backRightCurrent);

                double frontIntakeCurrent = intake1.getCurrentDraw();
                double backIntakeCurrent = intake2.getCurrentDraw();
                double shooter1Current = shooter1.getCurrentDraw();
                double shooter2Current = shooter2.getCurrentDraw();

                telemetry.addData("front intake current", frontIntakeCurrent);
                telemetry.addData("back intake current", backIntakeCurrent);
                telemetry.addData("shooter 1 current", shooter1Current);
                telemetry.addData("shooter2 current", shooter2Current);

                totalCurrent = frontLeftCurrent+frontRightCurrent+backLeftCurrent+backRightCurrent+frontIntakeCurrent+backIntakeCurrent+shooter1Current+shooter2Current;

                telemetry.addData("total current", totalCurrent);
            }


            long currentTime = System.nanoTime();
            double loopTime = (double) (currentTime - lastLoopTime) / 1_000_000.0;
            lastLoopTime = currentTime;
            telemetry.addData("Loop time", loopTime);
            intakes.update();
            spindexer.update();
            PanelsDrawing.drawDebug(follower);
            shooter.update();
            telemetry.update();
        }
    }
}
