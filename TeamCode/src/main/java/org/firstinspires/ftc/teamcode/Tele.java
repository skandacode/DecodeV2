package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pedro.Constants.createFollower;

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

import org.firstinspires.ftc.teamcode.pedro.PanelsDrawing;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Light;
import org.firstinspires.ftc.teamcode.subsystems.vision.LimelightCamera;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

import java.util.Arrays;
import java.util.List;

import org.firstinspires.ftc.teamcode.utils.CachedMotor;
import solverslib.gamepad.GamepadEx;
import solverslib.gamepad.GamepadKeys;

@Configurable
@TeleOp
public class Tele extends LinearOpMode {
    Intake intake;
    Transfer transfer;
    Shooter shooter;
    Follower follower;
    Light light;
    LimelightCamera limelight;

    CachedMotor frontLeft, frontRight, backLeft, backRight;
    CachedMotor intake1, intake2, shooter1, shooter2;

    public static Shooter.Goal target = Shooter.Goal.BLUE;
    public static double powerOffsetIncrements = 20;
    public static double turretOffsetIncrements = 2;

    public static double pulseTime = 0.05;


    public Pose relocalizePos = new Pose(-14.5, -56, Math.toRadians(-90));

    public static boolean allianceBlue = true;
    public static boolean telemetryCurrent = false;

    public enum States{
        Intake,
        TransferOff,
        BeforePulseOut,
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

        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        shooter = new Shooter(hardwareMap);
        follower = createFollower(hardwareMap);
        tilt = new Tilt(hardwareMap);
        limelight = new LimelightCamera(hardwareMap);
        light = new Light(hardwareMap);


        frontLeft = new CachedMotor(hardwareMap, "frontleft");
        frontRight = new CachedMotor(hardwareMap, "frontright");
        backLeft = new CachedMotor(hardwareMap, "backleft");
        backRight = new CachedMotor(hardwareMap, "backright");

        intake1 = new CachedMotor(hardwareMap, "frontIntake");
        intake2 = new CachedMotor(hardwareMap, "transferIntake");

        shooter1 = new CachedMotor(hardwareMap, "shooterMotor1");
        shooter2 = new CachedMotor(hardwareMap, "shooterMotor2");

        GamepadKeys.Button slowModeButton = GamepadKeys.Button.RIGHT_BUMPER;
        GamepadKeys.Button positionResetButton = GamepadKeys.Button.LEFT_BUMPER;

        GamepadKeys.Button shooterButton = GamepadKeys.Button.B;
        GamepadKeys.Button stopIntakeButton = GamepadKeys.Button.A;
        GamepadKeys.Button restartIntake = GamepadKeys.Button.Y;
        GamepadKeys.Button limelightAdjust = GamepadKeys.Button.X;

        GamepadKeys.Button tiltButton = GamepadKeys.Button.OPTIONS;

        follower.setStartingPose(Robot.endPose);

        StateMachine stateMachine = new StateMachineBuilder()
                .state(States.Intake)
                .onEnter(() -> {
                    light.setRed();
                    intake.setPower(1);
                    shooter.setUpperGate(false);
                    transfer.setLowerGate(true);
                    transfer.setKicker(false);
                    transfer.setPosition(Transfer.SpindexerPosition.Shoot0);
                })
                .transition(() -> gamepadEx.getButton(shooterButton), States.OpenUpperGate)
                .transition(() -> intake.getBeamBreakInside() && intake.getDetected(), States.TransferOff)
                .transition(() -> gamepadEx.getButton(stopIntakeButton), States.HoldBalls)

                .state(States.TransferOff)
                .onEnter(() -> intake.setTransferPower(0.3))
                .transition(() -> gamepadEx.getButton(stopIntakeButton), States.HoldBalls)
                .transition(() -> intake.getBeamBreakOutside() && intake.getBeamBreakInside(), States.BeforePulseOut)
                .transition(() -> gamepadEx.getButton(shooterButton), States.OpenUpperGate)

                .state(States.BeforePulseOut)
                .onEnter(() -> intake.setIntakePower(1))
                .transitionTimed(0.3)
                .transition(() -> gamepadEx.getButton(shooterButton), States.OpenUpperGate)

                .state(States.PulseOut)
                .onEnter(() -> intake.setIntakePower(-0.1))
                .transitionTimed(pulseTime)
                .transition(() -> gamepadEx.getButton(shooterButton), States.OpenUpperGate)

                .state(States.PulseIn)
                .onEnter(() -> intake.setIntakePower(1))
                .transitionTimed(0.2)
                .transition(() -> gamepadEx.getButton(shooterButton), States.OpenUpperGate)

                .state(States.HoldBalls)
                .onEnter(() -> intake.setPower(0.1))
                .loop(() -> {
                    if ((intake.getBeamBreakOutside() && intake.getBeamBreakInside())) {
                        intake.setPower(0.1);
                        light.setGreen();
                    } else {
                        intake.setPower(1);
                        light.setOrange();
                    }
                    if (!shooter.canReachPos){
                        light.setRed();
                    }
                })
                .transition(() -> gamepadEx.getButton(shooterButton), States.OpenUpperGate)
                .transition(() -> gamepadEx.getButton(restartIntake), States.Intake)

                .state(States.OpenUpperGate)
                .onEnter(() -> {
                    shooter.setUpperGate(true);
                    intake.setPower(1);
                })
                .transitionTimed(0.04, States.Shoot)
                .state(States.Shoot)
                .onEnter(() -> {
                    transfer.setKicker(true);
                })
                .transitionTimed(0.35, States.Intake)
                .build();

        while (opModeInInit()) {
            for (LynxModule hub : allHubs) hub.clearBulkCache();
            follower.update();
            if (gamepad1.a) {
                target = Shooter.Goal.BLUE;
                allianceBlue = true;
                relocalizePos = new Pose(-14.5, -56, Math.toRadians(-90));
            }
            if (gamepad1.b) {
                target = Shooter.Goal.RED;
                allianceBlue = false;
                relocalizePos = new Pose(-14.5, 56, Math.toRadians(90));
            }

            telemetry.addData("Shooter Target", target);
            telemetry.addData("Current Pos", follower.getPose());
            telemetry.update();
        }
        if (opModeIsActive()) {
            if (target == Shooter.Goal.BLUE) {
                limelight.setCurrentPipeline(LimelightCamera.Pipelines.BLUETRACK);
            } else {
                limelight.setCurrentPipeline(LimelightCamera.Pipelines.REDTRACK);
            }

            waitForStart();

            stateMachine.start();
            follower.startTeleopDrive();

            long lastLoopTime = System.nanoTime();
            boolean tilted = false;

            tilt.retract();
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

                follower.setTeleOpDrive(forward, -1 * strafe, -1 * turn, true);

                if (gamepadEx.wasJustPressed(positionResetButton)) {
                    follower.setPose(relocalizePos);
                    Shooter.limelightOffset = 0;
                    if (allianceBlue) {
                        Shooter.powerOffset = 0;
                        Shooter.turretOffset = 0;
                    } else {
                        Shooter.powerOffset = 0;
                        Shooter.turretOffset = 2;
                    }
                }
                if (gamepadEx.wasJustPressed(limelightAdjust)) {
                    Shooter.limelightOffset += limelight.getTrackingResults();
                }

                if (gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                    Shooter.powerOffset -= powerOffsetIncrements;
                }
                if (gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                    Shooter.turretOffset -= turretOffsetIncrements;
                }
                if (gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                    Shooter.turretOffset += turretOffsetIncrements;
                }
                if (gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                    Shooter.powerOffset += powerOffsetIncrements;
                }
                if (gamepadEx.wasJustPressed(tiltButton)) {
                    tilted = !tilted;
                    if (tilted) {
                        tilt.tilt();
                    } else {
                        tilt.retract();
                    }
                }
                stateMachine.update();

                telemetry.addData("Current Pos", follower.getPose());
                telemetry.addData("Shooter Target", shooter.getTargetVelo());
                telemetry.addData("Shooter Velocity", shooter.getCurrentVelocity());
                telemetry.addData("Spindexer kick", transfer.kicked);
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

                    totalCurrent = frontLeftCurrent + frontRightCurrent + backLeftCurrent + backRightCurrent + frontIntakeCurrent + backIntakeCurrent + shooter1Current + shooter2Current;

                    telemetry.addData("total current", totalCurrent);
                }


                long currentTime = System.nanoTime();
                double loopTime = (double) (currentTime - lastLoopTime) / 1_000_000.0;
                lastLoopTime = currentTime;
                telemetry.addData("Loop time", loopTime);
                intake.update();
                transfer.update();
                tilt.update();
                light.update();

                PanelsDrawing.drawDebug(follower);
                shooter.update();
                telemetry.update();
            }
        }
    }
}
