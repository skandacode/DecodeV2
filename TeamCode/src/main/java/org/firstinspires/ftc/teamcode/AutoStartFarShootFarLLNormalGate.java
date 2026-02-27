package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.subsystems.Intakes;
import org.firstinspires.ftc.teamcode.subsystems.LLFieldScannerResults;
import org.firstinspires.ftc.teamcode.subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.subsystems.Position;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

import java.util.List;

@Configurable
@Autonomous(name = "AutoFarFarLLNormalGate", group = "Auto")
public class AutoStartFarShootFarLLNormalGate extends LinearOpMode {
    private Follower follower;
    Intakes intakes;
    String colorAlliance = "BLUE";
    int Posmultiplier = 1;
    Shooter shooter;
    LimelightCamera limelightCamera;
    Spindexer spindexer;
    public boolean shooterButton = false;
    public double shootWaitTime = 0.25;


    public static Shooter.Goal shooterTarget = Shooter.Goal.BLUE;


    public enum AutoStates {
        MOVETOSHOOT1, wait1, preSHOOT1,SHOOT1,
        MOVETOINTAKE1,
        MOVETOSHOOT2, wait2, preSHOOT2,SHOOT2,
        MOVETOINTAKE2,
        MOVETOSHOOT3, wait3,preSHOOT3, SHOOT3,
        MOVETOINTAKE3,
        MOVETOSHOOT4, wait4,preSHOOT4, SHOOT4,
        MOVETOINTAKE4,
        MOVETOSHOOT5, wait5,preSHOOT5, SHOOT5,
        MOVETOINTAKE5,
        MOVETOSHOOT6, wait6,preSHOOT6, SHOOT6,
        MOVETOINTAKE6,
        MOVETOSHOOT7, wait7,preSHOOT7, SHOOT7,
        park
    }
    public enum States{
        Intake,
        Wait1,
        Increment2, //switch from Intake2 to Intake3
        Wait2,
        Increment3, //switch from Intake3 to 4
        WaitForShoot,
        Kick1,
        ShootSpin1,
        Kick2,
        ShootSpin2,
        Kick3,
        OpenUpperGate,
        Have3WaitShoot,
    }


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs)
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        intakes = new Intakes(hardwareMap);
        shooter = new Shooter(hardwareMap);
        spindexer = new Spindexer(hardwareMap);
        follower = createFollower(hardwareMap);
        limelightCamera = new LimelightCamera(hardwareMap);

        while (opModeInInit()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();
            follower.update();
            telemetry.addData("Init Pose: ", follower.getPose());
            telemetry.addData("ALLIANCE: ", colorAlliance);
            if (gamepad1.a){
                colorAlliance="BLUE";
                shooterTarget = Shooter.Goal.BLUE;
                Posmultiplier=1;
            }
            if (gamepad1.b){
                colorAlliance="RED";
                shooterTarget = Shooter.Goal.RED;
                Posmultiplier=-1;
            }
            spindexer.setPosition(Spindexer.SpindexerPosition.Shoot2);
            spindexer.setKickerPos(false);
            shooter.setTurretPos(shooter.convertDegreestoServoPos(0));
            shooter.setUpperGateOpen(false);
            spindexer.setLowerGateOpen(true);
            shooter.update();
            telemetry.update();
            spindexer.update();
        }

        waitForStart();


        Pose startPose = new Pose(60, -15*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose shootPose = new Pose(57, -19*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose intakeHuman = new Pose(58, -63*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose intake1Pose = new Pose(31, -17*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose intake1donePose = new Pose(31, -63*Posmultiplier, Math.toRadians(-90*Posmultiplier));


        Pose intakemidPose = new Pose(47, -17*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose intakemiddonePose = new Pose(47, -63*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose leave = new Pose(40, -30*Posmultiplier, Math.toRadians(-90*Posmultiplier));



        follower.setStartingPose(startPose);

        StateMachine autoMachine = new StateMachineBuilder() //Autonomia
                .transition(()->follower.atParametricEnd())
                .transitionTimed(0.4)
                .state(AutoStates.MOVETOSHOOT1)
                .onEnter(()->{
                    intakes.setBadIntakePower(-0.2);
                    intakes.setGoodIntakePower(1);
                    spindexer.setKickerPos(false);
                    PathChain toScore = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), shootPose))
                            .setLinearHeadingInterpolation(follower.getHeading(),shootPose.getHeading())
                            .setBrakingStrength(3)
                            .build();
                    follower.followPath(toScore, true);
                    shooter.setHood(0.8);
                    shooter.setTargetVelocity(2030);
                    shooter.setTurretPos(shooter.convertDegreestoServoPos(67*Posmultiplier));
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(0.5)

                .state(AutoStates.wait1)
                .onEnter(()->{
                    shooter.aimAtTarget(follower.getPose(),shooterTarget);
                })
                .transition(()->Math.abs(shooter.getTargetVelo()-shooter.getCurrentVelocity())<20)
                .transitionTimed(2.5)
                .state(AutoStates.preSHOOT1)
                .onEnter(()->{
                    shooter.setUpperGateOpen(true);
                })
                .transitionTimed(0.2)
                .state(AutoStates.SHOOT1)
                .onEnter(()->{
                    spindexer.setKickerPos(true);
                })
                .transitionTimed(0.5)


                .state(AutoStates.MOVETOINTAKE1)
                .onEnter(()->{
                    spindexer.setKickerPos(false);
                    shooter.setUpperGateOpen(false);
                    shooter.setTurretPos(shooter.convertDegreestoServoPos(67 *Posmultiplier));
                    shooter.setHood(0.795);
                    shooter.setTargetVelocity(1960);

                    PathChain toIntake = follower.pathBuilder()
                            .addPath(new BezierCurve(follower.getPose(), intake1Pose, intake1donePose))
                            .setLinearHeadingInterpolation(follower.getHeading(),intake1donePose.getHeading())
                            .setBrakingStrength(3)
                            .build();
                    follower.followPath(toIntake, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.6)

                .state(AutoStates.MOVETOSHOOT2)
                .onEnter(()->{
                    PathChain toScore = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), shootPose))
                            .setLinearHeadingInterpolation(follower.getHeading(),shootPose.getHeading())
                            .setBrakingStrength(3)
                            .build();
                    follower.followPath(toScore, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(2.5)

                .state(AutoStates.wait2)
                .onEnter(()->{
                    shooter.aimAtTarget(follower.getPose(),shooterTarget);
                })
                .transitionTimed(0.2)

                .state(AutoStates.preSHOOT2)
                .onEnter(()->{
                    shooter.setUpperGateOpen(true);
                })
                .transitionTimed(0.2)
                .state(AutoStates.SHOOT2)
                .onEnter(()->{
                    spindexer.setKickerPos(true);
                })
                .transitionTimed(0.5)
                .state(AutoStates.MOVETOINTAKE2)
                .onEnter(()->{
                    shooter.setUpperGateOpen(false);
                    spindexer.setKickerPos(false);
                    shooter.setTargetVelocity(2000);

                    shooter.setTurretPos(shooter.convertDegreestoServoPos(64*Posmultiplier));

                    PathChain toIntakeHuman = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), intakeHuman))
                            .setLinearHeadingInterpolation(follower.getHeading(),intakeHuman.getHeading())
                            .build();
                    follower.followPath(toIntakeHuman, true);
                })
                .transitionTimed(1.7)

                .state(AutoStates.MOVETOSHOOT3)
                .onEnter(()->{
                    PathChain toScore = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), shootPose))
                            .setLinearHeadingInterpolation(follower.getHeading(),shootPose.getHeading())
                            .setBrakingStrength(3)
                            .build();
                    follower.followPath(toScore, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(2.5)

                .state(AutoStates.wait3)
                .onEnter(()->{
                    shooter.aimAtTarget(follower.getPose(),shooterTarget);
                })
                .transitionTimed(0.2)

                .state(AutoStates.preSHOOT3)
                .onEnter(()->{
                    shooter.setUpperGateOpen(true);
                })
                .transitionTimed(0.2)
                .state(AutoStates.SHOOT3)
                .onEnter(()->{
                    spindexer.setKickerPos(true);
                })
                .transitionTimed(0.5)
                .state(AutoStates.MOVETOINTAKE3)
                .onEnter(()->{
                    shooter.setTurretPos(shooter.convertDegreestoServoPos(64*Posmultiplier));
                    shooter.setUpperGateOpen(false);
                    spindexer.setKickerPos(false);
                    LLFieldScannerResults results = limelightCamera.getTrackingResults();
                    if (results != null) {
                        double dx = results.getPosition()[1];

                        if (Posmultiplier == 1) {
                            dx = Math.max(dx, -3);
                        } else {
                            dx = Math.min(dx, 3);
                        }
                        Pose intakePoseAuto = new Pose(follower.getPose().getX() - Posmultiplier * dx, -63 * Posmultiplier, Math.toRadians(-90 * Posmultiplier));
                        PathChain toIntake = follower.pathBuilder()
                                .addPath(new BezierLine(follower.getPose(), intakePoseAuto))
                                .setLinearHeadingInterpolation(follower.getHeading(), intakePoseAuto.getHeading())
                                .build();
                        follower.followPath(toIntake, true);
                        System.out.println("Detected at " + dx);

                    }else{
                        PathChain toIntake = follower.pathBuilder()
                                .addPath(new BezierLine(follower.getPose(), intakeHuman))
                                .setLinearHeadingInterpolation(follower.getHeading(),intakeHuman.getHeading())
                                .build();
                        follower.followPath(toIntake, true);
                        System.out.println("Detected at NULL");

                    }

                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.8)
                .state(AutoStates.MOVETOSHOOT4)
                .onEnter(()->{
                    PathChain toScore = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), shootPose))
                            .setLinearHeadingInterpolation(follower.getHeading(),shootPose.getHeading())
                            .setBrakingStrength(3)
                            .build();
                    follower.followPath(toScore, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(2.5)

                .state(AutoStates.wait4)
                .onEnter(()->{
                    shooter.aimAtTarget(follower.getPose(),shooterTarget);
                })
                .transitionTimed(0.2)

                .state(AutoStates.preSHOOT4)
                .onEnter(()->{
                    shooter.setUpperGateOpen(true);
                })
                .transitionTimed(0.2)
                .state(AutoStates.SHOOT4)
                .onEnter(()->{
                    spindexer.setKickerPos(true);
                })
                .transitionTimed(0.5)
                .state(AutoStates.MOVETOINTAKE4)
                .onEnter(()->{
                    shooter.setTurretPos(shooter.convertDegreestoServoPos(64*Posmultiplier));
                    shooter.setUpperGateOpen(false);
                    spindexer.setKickerPos(false);
                    PathChain toIntake = follower.pathBuilder()
                            .addPath(new BezierCurve(follower.getPose(), intakemidPose, intakemiddonePose))
                            .setLinearHeadingInterpolation(follower.getHeading(),intakemiddonePose.getHeading())
                            .build();
                    follower.followPath(toIntake, true);

                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.8)
                .state(AutoStates.MOVETOSHOOT5)
                .onEnter(()->{
                    PathChain toScore = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), shootPose))
                            .setLinearHeadingInterpolation(follower.getHeading(),shootPose.getHeading())
                            .setBrakingStrength(0.7)
                            .build();
                    follower.followPath(toScore, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(2.5)

                .state(AutoStates.wait5)
                .onEnter(()->{
                    shooter.aimAtTarget(follower.getPose(),shooterTarget);
                })
                .transitionTimed(0.2)

                .state(AutoStates.preSHOOT5)
                .onEnter(()->{
                    shooter.setUpperGateOpen(true);
                })
                .transitionTimed(0.2)
                .state(AutoStates.SHOOT5)
                .onEnter(()->{
                    spindexer.setKickerPos(true);
                })
                .transitionTimed(0.5)
                .state(AutoStates.MOVETOINTAKE5)
                .onEnter(()->{
                    shooter.setTargetVelocity(2020);
                    shooter.setTurretPos(shooter.convertDegreestoServoPos(64*Posmultiplier));
                    shooter.setUpperGateOpen(false);
                    spindexer.setKickerPos(false);
                    LLFieldScannerResults results = limelightCamera.getTrackingResults();
                    if (results != null) {
                        double dx = results.getPosition()[1];

                        if (Posmultiplier == 1) {
                            dx = Math.max(dx, -3);
                        } else {
                            dx = Math.min(dx, 3);
                        }
                        Pose intakePoseAuto = new Pose(follower.getPose().getX() - Posmultiplier * dx, -63 * Posmultiplier, Math.toRadians(-90 * Posmultiplier));
                        PathChain toIntake = follower.pathBuilder()
                                .addPath(new BezierLine(follower.getPose(), intakePoseAuto))
                                .setLinearHeadingInterpolation(follower.getHeading(), intakePoseAuto.getHeading())
                                .build();
                        follower.followPath(toIntake, true);
                        System.out.println("Detected at " + dx);

                    }else{
                        PathChain toIntake = follower.pathBuilder()
                                .addPath(new BezierLine(follower.getPose(), intakeHuman))
                                .setLinearHeadingInterpolation(follower.getHeading(),intakeHuman.getHeading())
                                .build();
                        follower.followPath(toIntake, true);
                        System.out.println("Detected at NULL");

                    }

                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(2)

                .state(AutoStates.MOVETOSHOOT6)
                .onEnter(()->{
                    PathChain toScore = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), shootPose))
                            .setLinearHeadingInterpolation(follower.getHeading(),shootPose.getHeading())
                            .setBrakingStrength(0.7)
                            .build();
                    shooter.setTurretPos(shooter.convertDegreestoServoPos(64*Posmultiplier));

                    follower.followPath(toScore, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(2.5)

                .state(AutoStates.wait6)
                .onEnter(()->{
                    shooter.aimAtTarget(follower.getPose(),shooterTarget);
                })
                .transitionTimed(0.4)

                .state(AutoStates.preSHOOT6)
                .onEnter(()->{
                    shooter.setUpperGateOpen(true);
                })
                .transitionTimed(0.2)
                .state(AutoStates.SHOOT6)
                .onEnter(()->{
                    spindexer.setKickerPos(true);
                })
                .transitionTimed(0.5)
                .state(AutoStates.MOVETOINTAKE6)
                .onEnter(()->{
                    shooter.setUpperGateOpen(false);
                    spindexer.setKickerPos(false);
                    PathChain toIntake = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(),intakeHuman))
                            .setLinearHeadingInterpolation(follower.getHeading(),intakeHuman.getHeading())
                            .build();
                    follower.followPath(toIntake, true);

                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.8)
                .state(AutoStates.MOVETOSHOOT7)
                .onEnter(()->{
                    PathChain toScore = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), shootPose))
                            .setLinearHeadingInterpolation(follower.getHeading(),shootPose.getHeading())
                            .setBrakingStrength(0.7)
                            .build();
                    shooter.setTurretPos(shooter.convertDegreestoServoPos(64*Posmultiplier));

                    follower.followPath(toScore, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(2.5)

                .state(AutoStates.wait7)
                .onEnter(()->{
                    shooter.aimAtTarget(follower.getPose(),shooterTarget);
                })
                .transitionTimed(0.4)

                .state(AutoStates.preSHOOT7)
                .onEnter(()->{
                    shooter.setUpperGateOpen(true);
                })
                .transitionTimed(0.2)
                .state(AutoStates.SHOOT7)
                .onEnter(()->{
                    spindexer.setKickerPos(true);
                })
                .transitionTimed(0.5)
                .state(AutoStates.park)
                .onEnter(()->{
                    shooter.setUpperGateOpen(false);
                    spindexer.setKickerPos(false);
                    PathChain park = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), leave))
                            .setLinearHeadingInterpolation(follower.getHeading(),leave.getHeading())
                            .setBrakingStrength(0.6)
                            .build();
                    follower.followPath(park, true);
                })
                .build();

        autoMachine.start();
        limelightCamera.setCurrentPipeline(LimelightCamera.Pipelines.BALLTRACKING);
        while (opModeIsActive()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();
            Position.pose = follower.getPose();
            autoMachine.update();
            follower.update();
            intakes.update();
            shooter.update();
            spindexer.update();
            telemetry.addData("State auto: ", autoMachine.getState());
            telemetry.addData("Shooter Target", shooter.getTargetVelo());
            telemetry.addData("Shooter Velocity", shooter.getCurrentVelocity());
            telemetry.addData("Spindexer kick", spindexer.is_kick);
            telemetry.addData("Pose: ", follower.getPose());
            telemetry.update();
        }
    }
}

