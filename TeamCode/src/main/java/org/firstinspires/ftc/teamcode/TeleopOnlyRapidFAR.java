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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.pedroPathing.PanelsDrawing;
import org.firstinspires.ftc.teamcode.subsystems.Intakes;
import org.firstinspires.ftc.teamcode.subsystems.LEDIndicator;
import org.firstinspires.ftc.teamcode.subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.subsystems.Position;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.Tilt;

import java.util.Arrays;
import java.util.List;

import solverslib.controller.PIDFController;
import solverslib.gamepad.GamepadEx;
import solverslib.gamepad.GamepadKeys;
import solverslib.hardware.motors.Motor;

@Configurable
@TeleOp
public class TeleopOnlyRapidFAR extends LinearOpMode {
    Intakes intakes;
    Spindexer spindexer;
    Shooter shooter;
    Follower follower;
    Tilt tilt;
    LimelightCamera limelight;
    LEDIndicator indicator;

    Motor frontLeft, frontRight, backLeft, backRight;
    Motor intake1, intake2, shooter1, shooter2;

    public static Shooter.Goal target = Shooter.Goal.BLUE;
    public static double powerOffsetIncrements = 20;
    public static double turretOffsetIncrements = 2;
    public static double stallIntakeTime = 0.15;
    public static double openGateTime = 0.6;
    public static double intakeShooterVelo = 0.8;
    public static double tagToCenterOffset = -2;

    PIDFController turretDamper = new PIDFController(0.15, 0, 0, 0);


    public Pose relocalizePos = new Pose(58, 59, Math.toRadians(90));

    public static boolean allianceBlue = true;
    public static boolean telemetryCurrent = false;

    public enum States{
        Intake,
        PreShoot,
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
        limelight = new LimelightCamera(hardwareMap);
        indicator = new LEDIndicator(hardwareMap);


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
        GamepadKeys.Trigger unlockhood = GamepadKeys.Trigger.LEFT_TRIGGER;

        GamepadKeys.Button shooterButton = GamepadKeys.Button.B;
        GamepadKeys.Button stopIntakeButton = GamepadKeys.Button.A;
        GamepadKeys.Button restartIntake = GamepadKeys.Button.Y;
        GamepadKeys.Button limelightAdjust = GamepadKeys.Button.X;

        GamepadKeys.Button tiltButton = GamepadKeys.Button.OPTIONS;

        follower.setStartingPose(Position.pose);

        StateMachine stateMachine = new StateMachineBuilder()
                .state(States.Intake)
                .onEnter(() -> {
                    indicator.setRed();
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
                    if (intakes.getGoodBeamBreakOutside() && intakes.getGoodBeamBreakInside() && intakes.getGoodIntakeDetected()){
                        indicator.setGreen();
                    }else{
                        indicator.setOrange();
                    }
                    if (!shooter.canReachPos){
                        indicator.setRed();
                    }
                })
                .transition(() -> gamepadEx.getButton(shooterButton), States.OpenUpperGate)
                .state(States.OpenUpperGate)
                .onEnter(() -> {
                    shooter.setUpperGateOpen(true);

                })
                .transitionTimed(stallIntakeTime, States.PreShoot)
                .state(States.PreShoot)
                .onEnter(() -> {
                    intakes.setGoodIntakePower(intakeShooterVelo);
                })
                .transitionTimed(0.1, States.Shoot)
                .state(States.Shoot)
                .onEnter(() -> {
                    spindexer.setKickerPos(true);
                })
                .transitionTimed(openGateTime, States.Intake)
                .build();


        while (opModeInInit()) {
            for (LynxModule hub : allHubs) hub.clearBulkCache();
            follower.update();
            if (gamepad1.a) {
                target = Shooter.Goal.BLUE;
                allianceBlue = true;
                relocalizePos = new Pose(58, 58, Math.toRadians(90));

            }
            if (gamepad1.b) {
                target = Shooter.Goal.RED;
                allianceBlue = false;
                relocalizePos = new Pose(58, -58, Math.toRadians(-90));
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
            int counter = 0;
            ElapsedTime resetTimer = new ElapsedTime();

            tilt.retract();
            while (opModeIsActive()) {
                for (LynxModule hub : allHubs) {
                    hub.clearBulkCache();
                }
                gamepadEx.readButtons();
                follower.update();
                Position.pose = follower.getPose();
                telemetry.addData("Angle and distance:", Arrays.toString(shooter.getAngleDistance(Position.pose, target)));
                shooter.aimAtTargetFar(Position.pose, target);


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
                        Shooter.turretOffset = 0;
                    }
                }
                double tx = limelight.getTrackingResults();
                if (tx == 0){
                    counter+=1;
                    if (counter>60){
                        Shooter.limelightOffset=0;
                    }
                }else {
                    if (allianceBlue) {
                        tx += tagToCenterOffset;
                    }else{
                        tx -= tagToCenterOffset;
                    }
                    counter=0;
                    if (shooter.canReachPos) {
                        if (resetTimer.milliseconds() > 200) {
                            if (Math.abs(tx)>1) {
                                Shooter.limelightOffset -= turretDamper.calculate(tx, 0);
                            }
                            resetTimer.reset();
                        }
                    }
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
                telemetry.addData("Limelight Offset", Shooter.limelightOffset);
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

                    totalCurrent = frontLeftCurrent + frontRightCurrent + backLeftCurrent + backRightCurrent + frontIntakeCurrent + backIntakeCurrent + shooter1Current + shooter2Current;

                    telemetry.addData("total current", totalCurrent);
                }


                long currentTime = System.nanoTime();
                double loopTime = (double) (currentTime - lastLoopTime) / 1_000_000.0;
                lastLoopTime = currentTime;
                telemetry.addData("Loop time", loopTime);
                intakes.update();
                spindexer.update();
                tilt.update();
                indicator.update();

                PanelsDrawing.drawDebug(follower);
                shooter.update();
                telemetry.update();
            }
        }
    }
}
