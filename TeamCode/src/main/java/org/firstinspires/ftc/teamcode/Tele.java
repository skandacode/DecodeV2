package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.Arrays;

import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.firstinspires.ftc.teamcode.utils.PIDFController;

@Configurable
public class Tele extends OpMode {
    Robot robot;
    Alliance alliance;
    StateMachine stateMachine;

    public static Shooter.Goal target = Shooter.Goal.BLUE;
    public static double powerOffsetIncrements = 20;
    public static double hoodOffsetIncrements = 0.02;

    public static double turretOffsetIncrements = 2;

    public static double pulseTime = 0.05;

    public static double headingLock;
    private PIDFController headingPID, secondaryHeadingPID;


    public Pose relocalizePos;

    public enum States {
        Intake,
        BallsUp,
        TransferOff,
        BeforePulseOut,
        PulseOut,
        PulseIn,
        HoldBalls,
        OpenUpperGate,
        Shoot,
    }

    public Tele(Alliance alliance) {
        this.alliance = alliance;
    }

    @Override
    public void init() {
        if (alliance.equals(Alliance.BLUE)) {
            target = Shooter.Goal.BLUE;
            relocalizePos = new Pose(-14.5, -56, Math.toRadians(-90));
            Shooter.turretOffset=-2;
            Shooter.powerOffset=30;
            headingLock = -122;
        } else {
            target = Shooter.Goal.RED;
            relocalizePos = new Pose(-14.5, 56, Math.toRadians(90));
            headingLock = 122;
            Shooter.turretOffset=0;
            Shooter.powerOffset=0;
        }

        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        robot = new Robot(hardwareMap, alliance);
        robot.update();


        headingPID = new PIDFController(1.4426, 0, 0.2179, 0);
        secondaryHeadingPID = new PIDFController(0.6412, 0 ,0.1141, 0);

        robot.follower.setPose(Robot.savedPose);

        stateMachine = new StateMachineBuilder()
                .state(States.Intake)
                .onEnter(() -> {
                    robot.light.setRed();
                    robot.intake.setPower(1);
                    robot.shooter.setUpperGate(false);
                    robot.kicker.setKicker(false);
                })
                .transition(() -> gamepad1.bWasPressed(), States.OpenUpperGate)
                .transition(() -> robot.intake.getBeamBreakInside() && robot.intake.getDetected(), States.BallsUp)
                .transition(() -> gamepad1.aWasPressed(), States.HoldBalls)

                .state(States.BallsUp)
                .onEnter(()->{
                    robot.light.setYellow();
                    robot.intake.setPower(1);
                })
                .transitionTimed(0.3, States.TransferOff)
                .transition(() -> gamepad1.bWasPressed(), States.OpenUpperGate)

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
                .transitionTimed(0.5)
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
                    if (gamepad1.left_trigger_pressed){
                        robot.intake.setPower(0.7);
                    }else{
                        robot.intake.setPower(1);

                    }
                    robot.kicker.setKicker(true);
                })
                .transitionTimed(0.4, States.Intake)
                .build();
    }

    public void init_loop() {
        robot.follower.setPose(Robot.savedPose);
        robot.update();
        telemetry.addData("Shooter Target", target);
        telemetry.addData("Current Pos", robot.follower.pose());
        telemetry.update();
    }

    public void start() {
        stateMachine.start();
    }

    public void loop() {
        robot.update();
        telemetry.addData("Angle and distance:", Arrays.toString(robot.shooter.getAngleDistance(robot.follower.pose(), target)));
        robot.shooter.aimAtTarget(robot.follower.pose(), target);

        double forward = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        if (gamepad1.right_bumper) {
            forward *= 0.3;
            strafe *= 0.3;
            turn *= 0.3;
        }

        if (gamepad1.right_trigger > 0.1) {
            headingPID.setSetPoint(Angle.normalize(Math.toRadians(headingLock)));
            secondaryHeadingPID.setSetPoint(Angle.normalize(Math.toRadians(headingLock)));

            double error = Angle.normalize(Angle.normalize(Math.toRadians(headingLock)) - Angle.normalize(robot.follower.pose().heading()));
            double calc;

            if (Math.abs(error) > Math.PI/20)
                calc = headingPID.calculate(Angle.normalize(robot.follower.pose().heading()));
            else
                calc = secondaryHeadingPID.calculate(Angle.normalize(robot.follower.pose().heading()));

            telemetry.addData("heading lock enabled", calc);
            robot.follower.manual(-forward, -strafe, calc);
        } else {
            telemetry.addLine("heading lock disabled");
            robot.follower.manual(-forward, -strafe, -turn);
        }
        if (gamepad1.leftBumperWasPressed()) {
            robot.follower.setPose(relocalizePos);
            if (alliance.equals(Alliance.BLUE)) {
                Shooter.powerOffset = 0;
                Shooter.turretOffset = -2;
            } else {
                Shooter.powerOffset = 0;
                Shooter.turretOffset = 2;
            }
        }

        if (gamepad2.dpadDownWasPressed())
            Shooter.powerOffset -= powerOffsetIncrements;

        if (gamepad2.dpadLeftWasPressed())
            Shooter.turretOffset -= turretOffsetIncrements;
        if (gamepad2.dpadRightWasPressed())
            Shooter.turretOffset += turretOffsetIncrements;
        if (gamepad2.dpadUpWasPressed())
            Shooter.powerOffset += powerOffsetIncrements;
        if (gamepad2.yWasPressed())
            Shooter.hoodOffset -= hoodOffsetIncrements;
        if (gamepad2.aWasPressed())
            Shooter.hoodOffset += hoodOffsetIncrements;

        stateMachine.update();

        telemetry.addData("Current Pos", robot.follower.pose());
        telemetry.addData("Shooter Target", robot.shooter.getTargetVelo());
        telemetry.addData("Shooter Velocity", robot.shooter.getCurrentVelocity());
        telemetry.addData("Shooter power offset", Shooter.powerOffset);
        telemetry.addData("Shooter turret offset", Shooter.turretOffset);


        telemetry.addData("Spindexer kick", robot.kicker.kicked);
        telemetry.addData("Statemachine State", stateMachine.getState());
        telemetry.addData("Loop time hz", robot.getLoopTimeHz());

//        PanelsDrawing.drawDebug(robot.follower);
        telemetry.update();
    }
}