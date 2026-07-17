package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.pedro.PanelsDrawing;
import org.firstinspires.ftc.teamcode.subsystems.vision.LimelightCamera;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.Arrays;

import org.firstinspires.ftc.teamcode.utils.Alliance;
@Configurable
public class StateMachineTesting extends OpMode {
    Robot robot;
    Alliance alliance;
    StateMachine stateMachine;

    public static Shooter.Goal target = Shooter.Goal.BLUE;

    public static double pulseTime = 0.05;


    public Pose relocalizePos = new Pose(-14.5, -56, Math.toRadians(-90));

    public static boolean allianceBlue = true;
    public static boolean telemetryCurrent = false;

    public enum States {
        Intake,
        TransferOff,
        BeforePulseOut,
        PulseOut,
        PulseIn,
        HoldBalls,
        OpenUpperGate,
        Shoot,
    }

    public StateMachineTesting(Alliance alliance) {
        this.alliance = alliance;
    }

    @Override
    public void init() {
        if (alliance.equals(Alliance.BLUE)) {
            target = Shooter.Goal.BLUE;
            allianceBlue = true;
            relocalizePos = new Pose(-14.5, -56, Math.toRadians(-90));
        } else {
            target = Shooter.Goal.RED;
            allianceBlue = false;
            relocalizePos = new Pose(-14.5, 56, Math.toRadians(90));
        }

        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        robot = new Robot(hardwareMap, alliance, true);
        robot.update();

        robot.follower.setStartingPose();

        stateMachine = new StateMachineBuilder()
                .state(States.Intake)
                .onEnter(() -> {
                    robot.light.setRed();
                    robot.intake.setPower(1);
                    robot.shooter.setUpperGate(false);
                    robot.kicker.setKicker(false);
                })
                .transition(() -> gamepad1.bWasPressed(), States.OpenUpperGate)
                .transition(() -> robot.intake.getBeamBreakInside() && robot.intake.getDetected(), States.TransferOff)
                .transition(() -> gamepad1.aWasPressed(), States.HoldBalls)

                .state(States.TransferOff)
                .onEnter(() -> robot.intake.setTransferPower(0.3))
                .transition(() -> gamepad1.aWasPressed(), States.HoldBalls)
                .transition(() -> robot.intake.getBeamBreakOutside() && robot.intake.getBeamBreakInside(), States.BeforePulseOut)
                .transition(() -> gamepad1.bWasPressed(), States.OpenUpperGate)

                .state(States.BeforePulseOut)
                .onEnter(() -> robot.intake.setIntakePower(1))
                .transitionTimed(0.3)
                .transition(() -> gamepad1.bWasPressed(), States.OpenUpperGate)

                .state(States.PulseOut)
                .onEnter(() -> robot.intake.setIntakePower(-0.1))
                .transitionTimed(pulseTime)
                .transition(() -> gamepad1.bWasPressed(), States.OpenUpperGate)

                .state(States.PulseIn)
                .onEnter(() -> robot.intake.setIntakePower(1))
                .transitionTimed(0.2)
                .transition(() -> gamepad1.bWasPressed(), States.OpenUpperGate)

                .state(States.HoldBalls)
                .onEnter(() -> robot.intake.setPower(0.1))
                .loop(() -> {
                    if ((robot.intake.getBeamBreakOutside() && robot.intake.getBeamBreakInside())) {
                        robot.intake.setPower(0.1);
                        robot.light.setGreen();
                    } else {
                        robot.intake.setPower(1);
                        robot.light.setOrange();
                    }
                    if (!robot.shooter.canReachPos) {
                        robot.light.setRed();
                    }
                })
                .transition(() -> gamepad1.bWasPressed(), States.OpenUpperGate)
                .transition(() -> gamepad1.yWasPressed(), States.Intake)

                .state(States.OpenUpperGate)
                .onEnter(() -> {
                    robot.shooter.setUpperGate(true);
                    robot.intake.setPower(0);
                })
                .transitionTimed(0.1, States.Shoot)
                .state(States.Shoot)
                .onEnter(() -> {
                    robot.intake.setPower(1);
                    robot.kicker.setKicker(true);
                })
                .transitionTimed(0.3, States.Intake)
                .build();
    }

    public void init_loop() {
        robot.update();
//        if (gamepad1.a) {
//            target = Shooter.Goal.BLUE;
//            allianceBlue = true;
//            relocalizePos = new Pose(-14.5, -56, Math.toRadians(-90));
//        }
//        if (gamepad1.b) {
//            target = Shooter.Goal.RED;
//            allianceBlue = false;
//            relocalizePos = new Pose(-14.5, 56, Math.toRadians(90));
//        }

        telemetry.addData("Shooter Target", target);
        telemetry.addData("Current Pos", robot.follower.getPose());
        telemetry.update();
    }

    public void start() {
        if (target == Shooter.Goal.BLUE) {
            robot.limelight.setCurrentPipeline(LimelightCamera.Pipelines.BLUETRACK);
        } else {
            robot.limelight.setCurrentPipeline(LimelightCamera.Pipelines.REDTRACK);
        }

        stateMachine.start();
        robot.follower.startTeleopDrive();
    }

    public void loop() {
        robot.update();
        telemetry.addData("Angle and distance:", Arrays.toString(robot.shooter.getAngleDistance(robot.follower.getPose(), target)));
        robot.shooter.aimAtTarget(robot.follower.getPose(), target);

        double forward = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        if (gamepad1.right_bumper) {
            forward *= 0.3;
            strafe *= 0.3;
            turn *= 0.3;
        }

        robot.follower.setTeleOpDrive(forward, -1 * strafe, -1 * turn, true);

        if (gamepad1.leftBumperWasPressed()) {
            robot.follower.setPose(relocalizePos);
            Shooter.limelightOffset = 0;
            if (allianceBlue) {
                Shooter.powerOffset = 0;
                Shooter.turretOffset = 0;
            } else {
                Shooter.powerOffset = 0;
                Shooter.turretOffset = 2;
            }
        }

        if (gamepad1.xWasPressed())
            Shooter.limelightOffset += robot.limelight.getTrackingResults();


        if (gamepad1.dpadDownWasPressed())
            Shooter.powerOffset -= powerOffsetIncrements;

        if (gamepad1.dpadLeftWasPressed())
            Shooter.turretOffset -= turretOffsetIncrements;

        if (gamepad1.dpadRightWasPressed())
            Shooter.turretOffset += turretOffsetIncrements;
        if (gamepad1.dpadUpWasPressed())
            Shooter.powerOffset += powerOffsetIncrements;

        stateMachine.update();

        telemetry.addData("Current Pos", robot.follower.getPose());
        telemetry.addData("Shooter Target", robot.shooter.getTargetVelo());
        telemetry.addData("Shooter Velocity", robot.shooter.getCurrentVelocity());
        telemetry.addData("Spindexer kick", robot.kicker.kicked);
        telemetry.addData("Statemachine State", stateMachine.getState());
        telemetry.addData("Loop time hz", robot.getLoopTimeHz());

        PanelsDrawing.drawDebug(robot.follower);
        telemetry.update();
    }
}
