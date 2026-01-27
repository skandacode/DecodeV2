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
import org.firstinspires.ftc.teamcode.subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.subsystems.Position;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

import java.util.List;

@Configurable
@Autonomous(name = "AutoCloseGate", group = "Auto")
public class AutoStartCloseGate extends LinearOpMode {
    private Follower follower;
    public static int[] shootorder = {0, 1, 2};
    LimelightCamera limelightCamera;
    Intakes intakes;
    String colorAlliance = "BLUE";
    int Posmultiplier = 1;
    Shooter shooter;
    Spindexer spindexer;
    public int pattern = 1;
    public boolean shooterButton = false;
    public double shootWaitTime = 0.3;
    public static boolean rapidFire = true;

//Hello, humans.

    public static Shooter.Goal shooterTarget = Shooter.Goal.BLUE;


    public enum AutoStates {
        MOVETOSHOOT1, wait1, SHOOT1,
        MOVETOINTAKE1,
        MOVETOSHOOT2, wait2, SHOOT2,
        MOVETOINTAKE2,
        MOVETOSHOOT3, wait3,SHOOT3,
        MOVETOINTAKE3, backupgate1,forwardsgate,waitgate1,
        MOVETOSHOOT4, wait4,SHOOT4,
        MOVETOINTAKE4,backupgate2,waitgate2,
        MOVETOSHOOT5, wait5,SHOOT5,
        MOVETOINTAKE5,
        MOVETOSHOOT6, wait6,SHOOT6,
        MOVETOINTAKE6, INTAKE6BACK, INTAKE6IN,
        MOVETOSHOOT7, wait7,SHOOT7,
        LEAVE
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
        Shoot,
    }


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs)
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        limelightCamera = new LimelightCamera(hardwareMap);
        intakes = new Intakes(hardwareMap);
        shooter = new Shooter(hardwareMap);
        spindexer = new Spindexer(hardwareMap);
        follower = createFollower(hardwareMap);

        while (opModeInInit()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();
            follower.update();
            int currpattern = limelightCamera.getMotif();
            spindexer.setPosition(Spindexer.SpindexerPosition.Shoot1);
            if (currpattern != 0){
                pattern = currpattern;
            }else{
                telemetry.addLine("Don't see anything");
            }
            telemetry.addData("Pattern", pattern);
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
            telemetry.update();
            spindexer.update();
        }

        waitForStart();


        Pose gateintake = new Pose(15, -58*Posmultiplier, Math.toRadians(77*Posmultiplier));

        Pose gateintakeback = new Pose(22, -60*Posmultiplier, Math.toRadians(-140*Posmultiplier));

        Pose gateintakecontrol = new Pose(26, -42*Posmultiplier, Math.toRadians(77*Posmultiplier));
        Pose startPose = new Pose(-40, -55*Posmultiplier, Math.toRadians(-36.3*Posmultiplier));

        Pose shootPose = new Pose(-16, -16*Posmultiplier, Math.toRadians(-40*Posmultiplier));
        Pose shootPose2nd = new Pose(-0, -17*Posmultiplier, Math.toRadians(-50*Posmultiplier));
        Pose shootPosegoingtolast = new Pose(-4, -20*Posmultiplier, Math.toRadians(-23*Posmultiplier));
        Pose shootPoseleave = new Pose(-30, -16*Posmultiplier, Math.toRadians(-23*Posmultiplier));

        Pose intake1Pose = new Pose(-3, -27*Posmultiplier, Math.toRadians(-80*Posmultiplier));
        Pose intake2Pose = new Pose(23, -29*Posmultiplier, Math.toRadians(-80*Posmultiplier));
        Pose intake3Pose = new Pose(46,-20*Posmultiplier, Math.toRadians(-192*Posmultiplier));
        Pose intake3Poseback = new Pose(46,-40*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose intake4Pose = new Pose(33,-58*Posmultiplier, Math.toRadians(-15*Posmultiplier));
        Pose intake4donePose = new Pose(64, -62*Posmultiplier, Math.toRadians(0*Posmultiplier));
        Pose intake4Poseback = new Pose(55, -62*Posmultiplier, Math.toRadians(0*Posmultiplier));

        Pose intake1donePose = new Pose(-3, -55*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose intake2donePose = new Pose(23, -63*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose intake3donePose = new Pose(46, -63*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose leave = new Pose(60, -46*Posmultiplier, Math.toRadians(-90*Posmultiplier));


        PathChain toShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .setBrakingStrength(1.5)
                .build();
        PathChain toIntake1 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, intake1Pose, intake1donePose))
                .setTangentHeadingInterpolation()
                .setBrakingStrength(0.6)
                .build();

        PathChain toIntake2 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose2nd, intake2Pose, intake2donePose))
                .setTangentHeadingInterpolation()
                .setBrakingStrength(0.6)
                .build();

        PathChain toIntakeGate = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose2nd, gateintakecontrol, gateintake))
                .setTangentHeadingInterpolation()
                .build();

        PathChain toIntake3 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose2nd, intake3Pose,intake3donePose))
                .setTangentHeadingInterpolation()
                .setNoDeceleration()
                .build();
        PathChain toIntake4 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPosegoingtolast, intake4Pose,intake4donePose))
                .setNoDeceleration()
                .setTangentHeadingInterpolation()
                .build();
        PathChain toIntake4back = follower.pathBuilder()
                .addPath(new BezierLine(intake4donePose, intake4Poseback))
                .setConstantHeadingInterpolation(intake4donePose.getHeading())
                .setNoDeceleration()
                .build();
        PathChain toIntake4in = follower.pathBuilder()
                .addPath(new BezierLine(intake4Poseback, intake4donePose))
                .setConstantHeadingInterpolation(intake4donePose.getHeading())
                .setNoDeceleration()
                .build();


        PathChain toScore1 = follower.pathBuilder()
                .addPath(new BezierLine(intake1donePose, shootPose2nd))
                .setLinearHeadingInterpolation(intake1donePose.getHeading(), shootPose2nd.getHeading())
                .setBrakingStrength(1.6)
                .build();

        PathChain toScore2 = follower.pathBuilder()
                .addPath(new BezierLine(intake2donePose, shootPose2nd))
                .setBrakingStrength(4)
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        PathChain toScore3 = follower.pathBuilder()
                .addPath(new BezierLine(intake3donePose, shootPosegoingtolast))
                .setBrakingStrength(4)
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
        PathChain toScore4 = follower.pathBuilder()
                .addPath(new BezierLine(intake4donePose, shootPoseleave))
                .setBrakingStrength(3)
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
        PathChain toScoreGate = follower.pathBuilder()
                .addPath(new BezierLine(gateintakeback, shootPose2nd))
                .setLinearHeadingInterpolation(gateintakeback.getHeading(), shootPose2nd.getHeading())
                .build();



        follower.setStartingPose(startPose);




        StateMachine autoMachine = new StateMachineBuilder() //Autonomia
                .state(AutoStates.MOVETOSHOOT1)
                .onEnter(()->{
                    shooter.setUpperGateOpen(false);
                    spindexer.setLowerGateOpen(true);
                    intakes.setGoodIntakePower(1);
                    follower.followPath(toShoot, true);
                    shooter.setHood(0.49);
                    shooter.setTargetVelocity(1340);
                    shooter.setTurretPos(shooter.convertDegreestoServoPos(91.5*Posmultiplier));
                })

                .transitionTimed(1.5)
                .state(AutoStates.wait1)
                .onEnter(()->{
                    shooter.setUpperGateOpen(true);
                    spindexer.setLowerGateOpen(true);

                })
                .transitionTimed(0.3)
                .state(AutoStates.SHOOT1)
                .onEnter(()->{
                    spindexer.setKickerPos(true);
                })
                .transitionTimed(0.3)

                .state(AutoStates.MOVETOINTAKE1)
                .onEnter(()->{
                    shooter.setUpperGateOpen(false);
                    spindexer.setKickerPos(false);
                    follower.followPath(toIntake1, true);
                })

                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.4)

                .state(AutoStates.MOVETOSHOOT2)
                .onEnter(()->{
                    shooter.setTurretPos(shooter.convertDegreestoServoPos(100*Posmultiplier));
                    shooter.setHood(0.53);
                    shooter.setTargetVelocity(1400);
                    follower.followPath(toScore1, true);
                })
                .transition(()->!follower.isBusy())
                .transitionTimed(2)

                .state(AutoStates.wait2)
                .onEnter(()->{
                    shooter.setUpperGateOpen(true);
                })
                .transitionTimed(0.4)
                .state(AutoStates.SHOOT2)
                .onEnter(()->{
                    spindexer.setKickerPos(true);
                })
                .transitionTimed(0.2)
                .state(AutoStates.MOVETOINTAKE2)
                .onEnter(()->{
                    spindexer.setKickerPos(false);
                    shooter.setUpperGateOpen(false);
                    follower.followPath(toIntake2, true);
                })

                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.3)

                .state(AutoStates.MOVETOSHOOT3)
                .onEnter(()->{
                    shooter.setTurretPos(shooter.convertDegreestoServoPos(75*Posmultiplier));
                    shooter.setHood(0.54);
                    shooter.setTargetVelocity(1440);

                    follower.followPath(toScore2, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.5)

                .state(AutoStates.wait3)
                .onEnter(()->{
                    shooter.setUpperGateOpen(true);
                })
                .transitionTimed(0.3)
                .state(AutoStates.SHOOT3)
                .onEnter(()->{
                    spindexer.setKickerPos(true);
                    shooterButton=true;
                })

                .transitionTimed(0.2)

                .state(AutoStates.MOVETOINTAKE3)
                .onEnter(()->{
                    spindexer.setKickerPos(false);
                    shooter.setUpperGateOpen(false);
                    follower.followPath(toIntakeGate, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.3)
                .state(AutoStates.backupgate1)
                .onEnter(()->{
                    spindexer.setKickerPos(false);
                    shooter.setUpperGateOpen(false);
                    PathChain toIntakeBackGate = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), gateintakeback))
                            .setLinearHeadingInterpolation(follower.getHeading(), gateintakeback.getHeading())
                            .build();
                    follower.followPath(toIntakeBackGate, true);
                })
                .transitionTimed(0.8)
                .state(AutoStates.waitgate1)
                .onEnter(()->{

                })
                .transitionTimed(0.7)
                .state(AutoStates.MOVETOSHOOT4)
                .onEnter(()->{
                    shooter.setTurretPos(shooter.convertDegreestoServoPos(98*Posmultiplier));
                    shooter.setTargetVelocity(1410);
                    follower.followPath(toScoreGate, true);
                })

                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.7)
                .state(AutoStates.wait4)
                .onEnter(()->{
                    shooter.setUpperGateOpen(true);
                })
                .transitionTimed(0.4)
                .state(AutoStates.SHOOT4)
                .onEnter(()->{
                    spindexer.setKickerPos(true);
                })
                .transitionTimed(0.4)



                .state(AutoStates.MOVETOINTAKE4)
                .onEnter(()->{
                    spindexer.setKickerPos(false);
                    shooter.setUpperGateOpen(false);
                    follower.followPath(toIntakeGate, true);
                })
                .transitionTimed(1.8)

                .state(AutoStates.waitgate2)
                .onEnter(()->{

                    PathChain toIntakeBackoutGate = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), gateintakeback))
                            .setLinearHeadingInterpolation(follower.getHeading(), gateintakeback.getHeading())
                            .build();
                    follower.followPath(toIntakeBackoutGate, true);

                })
                .transitionTimed(1.256767)
                .state(AutoStates.MOVETOSHOOT5)
                .onEnter(()->{
                    shooter.setTurretPos(shooter.convertDegreestoServoPos(100*Posmultiplier));
                    PathChain toScorenobackgate = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), shootPose2nd))
                            .setLinearHeadingInterpolation(follower.getHeading(), shootPose2nd.getHeading())
                            .build();
                    follower.followPath(toScorenobackgate, true);
                })

                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.7)
                .state(AutoStates.wait5)
                .onEnter(()->{
                    shooter.setUpperGateOpen(true);
                })
                .transitionTimed(0.4)
                .state(AutoStates.SHOOT5)
                .onEnter(()->{
                    spindexer.setKickerPos(true);
                })
                .transitionTimed(0.2)



                .state(AutoStates.MOVETOINTAKE5)
                .onEnter(()->{
                    spindexer.setKickerPos(false);
                    shooter.setUpperGateOpen(false);
                    follower.followPath(toIntake3, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.8)

                .state(AutoStates.MOVETOSHOOT6)
                .onEnter(()->{
                    shooter.setTurretPos(shooter.convertDegreestoServoPos(99*Posmultiplier));
                    shooter.setHood(0.55);
                    shooter.setTargetVelocity(1380);
                    follower.followPath(toScore3, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(2)
                .state(AutoStates.wait6)
                .onEnter(()->{
                    shooter.setUpperGateOpen(true);
                })
                .transitionTimed(0.4)
                .state(AutoStates.SHOOT6)
                .onEnter(()->{
                    spindexer.setKickerPos(true);
                })
                .transitionTimed(0.4)


                .state(AutoStates.MOVETOINTAKE6)
                .onEnter(()->{
                    spindexer.setKickerPos(false);
                    shooter.setUpperGateOpen(false);
                    follower.followPath(toIntake4, true);
                })
                .transitionTimed(1.8)
                .state(AutoStates.INTAKE6BACK)
                .onEnter(()->{
                    follower.followPath(toIntake4back, true);
                })
                .transitionTimed(0.167)
                .state(AutoStates.INTAKE6IN)
                .onEnter(()->{
                    follower.followPath(toIntake4in, true);
                })
                .transitionTimed(0.4)

                .state(AutoStates.MOVETOSHOOT7)
                .onEnter(()->{
                    shooter.setTurretPos(shooter.convertDegreestoServoPos(93*Posmultiplier));
                    shooter.setTargetVelocity(1300);
                    shooter.setHood(0.5);
                    follower.followPath(toScore4, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(2.4)
                .state(AutoStates.wait7)
                .onEnter(()->{
                    shooter.setUpperGateOpen(true);
                })
                .transitionTimed(0.5)
                .state(AutoStates.SHOOT7)
                .onEnter(()->{
                    spindexer.setKickerPos(true);
                })
                .build();

        autoMachine.start();

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

