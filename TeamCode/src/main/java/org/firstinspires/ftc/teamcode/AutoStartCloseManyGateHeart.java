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

import org.firstinspires.ftc.teamcode.pedroPathing.PositionLogger;
import org.firstinspires.ftc.teamcode.subsystems.Intakes;
import org.firstinspires.ftc.teamcode.subsystems.Position;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

import java.io.IOException;
import java.util.List;

@Configurable
@Autonomous(name = "HeartClose18", group = "Auto")
public class AutoStartCloseManyGateHeart extends LinearOpMode {
    private Follower follower;
    Intakes intakes;
    String colorAlliance = "BLUE";
    int Posmultiplier = 1;
    Shooter shooter;
    Spindexer spindexer;
    public int pattern = 1;
    public boolean shooterButton = false;
    public static Shooter.Goal shooterTarget = Shooter.Goal.BLUE;
    PositionLogger positionLogger;


    public enum AutoStates {
        MOVETOSHOOT1, wait1, SHOOT1,
        MOVETOINTAKE1, back1, gate1,
        MOVETOSHOOT2, wait2, SHOOT2,
        MOVETOINTAKE2, back2, gate2,
        MOVETOSHOOT3, wait3,SHOOT3,
        MOVETOINTAKE3, backupgate1,upgate1,waitgate1,
        MOVETOSHOOT4, wait4,SHOOT4,
        MOVETOINTAKE4,backupgate2, upgate2, waitgate2,
        MOVETOSHOOT5, wait5,SHOOT5,
        MOVETOINTAKE5, backupgate3, upgate3,waitgate3,
        MOVETOSHOOT6, wait6,SHOOT6,
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
        try {
            positionLogger = new PositionLogger("Close 21 Auto.log");
        } catch (IOException e) {
            System.out.println("Logger failed to Initialize");
            positionLogger = null;
        }
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        shooterTarget = Shooter.Goal.BLUE;
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs)
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        intakes = new Intakes(hardwareMap);
        shooter = new Shooter(hardwareMap);
        spindexer = new Spindexer(hardwareMap);
        follower = createFollower(hardwareMap);

        while (opModeInInit()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();
            follower.update();
            spindexer.setPosition(Spindexer.SpindexerPosition.Shoot0);
            shooter.setTurretPos(shooter.convertDegreestoServoPos(0));

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
            shooter.update();
        }

        waitForStart();


        Pose gateopen = new Pose(-3, -59*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose gatepreopen = new Pose(-3, -30*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose gatepreopen2ndtime = new Pose(4, -30*Posmultiplier, Math.toRadians(-90*Posmultiplier));


        Pose gateintake1 = new Pose(9, -61*Posmultiplier, Math.toRadians(-110*Posmultiplier));
        Pose gateintakeopen1 = new Pose(8, -56*Posmultiplier, Math.toRadians(-80*Posmultiplier));
        Pose gateintake2 = new Pose(9, -61*Posmultiplier, Math.toRadians(-110*Posmultiplier));
        Pose gateintakeopen2 = new Pose(9, -56*Posmultiplier, Math.toRadians(-80*Posmultiplier));
        Pose gateintake3 = new Pose(7, -61*Posmultiplier, Math.toRadians(-110*Posmultiplier));
        Pose gateintakeopen3 = new Pose(9, -56*Posmultiplier, Math.toRadians(-80*Posmultiplier));
        Pose gateintake4 = new Pose(9, -61*Posmultiplier, Math.toRadians(-110*Posmultiplier));
        Pose gateintakeopen4 = new Pose(9, -56*Posmultiplier, Math.toRadians(-80*Posmultiplier));

        Pose gateintakecontrol = new Pose(8, -42*Posmultiplier, Math.toRadians(77*Posmultiplier));

        Pose startPose = new Pose(-44, -55*Posmultiplier, Math.toRadians(6*Posmultiplier));
        Pose startPoseOLD = new Pose(-37, -55.5*Posmultiplier, Math.toRadians(0*Posmultiplier));

        Pose shootPose = new Pose(-32, -16*Posmultiplier, Math.toRadians(-40*Posmultiplier));
        Pose shootPose2nd = new Pose(-17, -17*Posmultiplier, Math.toRadians(-50*Posmultiplier));
        Pose shootPosegoingtolast = new Pose(-20, -20*Posmultiplier, Math.toRadians(-23*Posmultiplier));
        Pose shootPoseleave = new Pose(-30, -16*Posmultiplier, Math.toRadians(-23*Posmultiplier));

        Pose intake1Pose = new Pose(-17, -27*Posmultiplier, Math.toRadians(-80*Posmultiplier));
        Pose intake2Pose = new Pose(13, -29*Posmultiplier, Math.toRadians(-80*Posmultiplier));
        Pose intake3Pose = new Pose(36,-20*Posmultiplier, Math.toRadians(-192*Posmultiplier));
        Pose intake3Poseback = new Pose(34,-40*Posmultiplier, Math.toRadians(-90*Posmultiplier));

        Pose intake1donePose = new Pose(-16, -52*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose intake2donePose = new Pose(10, -63*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose intake3donePose = new Pose(34, -63*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose leave = new Pose(-14, -55*Posmultiplier, Math.toRadians(-90*Posmultiplier));


        PathChain toShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .setBrakingStrength(1.5)
                .build();
        PathChain toIntake1 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, intake1Pose, intake1donePose))
                .setTangentHeadingInterpolation()
                .setNoDeceleration()
                .build();

        PathChain toIntake2 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose2nd, intake2Pose, intake2donePose))
                .setTangentHeadingInterpolation()
                .setNoDeceleration()
                .build();

        PathChain openGate2 = follower.pathBuilder()
                .addPath(new BezierLine(gatepreopen2ndtime,gateopen))
                .setLinearHeadingInterpolation(gatepreopen2ndtime.getHeading(),gateopen.getHeading())
                .build();
        PathChain openGate = follower.pathBuilder()
                .addPath(new BezierLine(gatepreopen,gateopen))
                .setLinearHeadingInterpolation(gatepreopen.getHeading(),gateopen.getHeading())
                .build();

        PathChain intake1back = follower.pathBuilder()
                .addPath(new BezierLine(intake1donePose,gatepreopen))
                .setNoDeceleration()
                .setLinearHeadingInterpolation(intake1donePose.getHeading(),gatepreopen.getHeading())
                .build();
        PathChain intake2back = follower.pathBuilder()
                .addPath(new BezierCurve(intake2donePose,intake2Pose,gatepreopen2ndtime))
                .setNoDeceleration()
                .setLinearHeadingInterpolation(intake2donePose.getHeading(),gatepreopen2ndtime.getHeading())
                .build();


        PathChain toScore1 = follower.pathBuilder()
                .addPath(new BezierLine(gateopen, shootPose2nd))
                .setLinearHeadingInterpolation(gateopen.getHeading(), shootPose2nd.getHeading())
                .setBrakingStrength(1.6)
                .build();

        PathChain toScore2 = follower.pathBuilder()
                .addPath(new BezierLine(gateopen, shootPose2nd))
                .setBrakingStrength(4)
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();






        follower.setStartingPose(startPose);




        StateMachine autoMachine = new StateMachineBuilder() //Autonomia
                .state(AutoStates.MOVETOSHOOT1)
                .onEnter(()->{
                    shooter.setUpperGateOpen(false);
                    spindexer.setLowerGateOpen(true);
                    intakes.setGoodIntakePower(1);
                    follower.followPath(toShoot, true);
                    shooter.setHood(0.53);
                    shooter.setTargetVelocity(1300);
                    if (Posmultiplier==1) {
                        shooter.setTurretPos(shooter.convertDegreestoServoPos(79));
                    }else{
                        shooter.setTurretPos(shooter.convertDegreestoServoPos(-86));
                    }
                })

                .transitionTimed(1.6)
                .state(AutoStates.wait1)
                .onEnter(()->{
                    shooter.setUpperGateOpen(true);

                })
                .transitionTimed(0.1)
                .state(AutoStates.SHOOT1)
                .onEnter(()->{
                    spindexer.setKickerPos(true);
                })
                .transitionTimed(0.6)

                .state(AutoStates.MOVETOINTAKE1)
                .onEnter(()->{
                    shooter.setUpperGateOpen(false);
                    spindexer.setKickerPos(false);
                    follower.followPath(toIntake1, true);
                })

                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.2)
                .state(AutoStates.back1)
                .onEnter(()->{
                    follower.followPath(intake1back, true);
                })

                .transition(()->follower.atParametricEnd())
                .transitionTimed(0.3)
                .state(AutoStates.gate1)
                .onEnter(()->{
                    follower.followPath(openGate, true);
                })

                .transitionTimed(1.2)

                .state(AutoStates.MOVETOSHOOT2)
                .onEnter(()->{
                    if (Posmultiplier==1) {
                        shooter.setTurretPos(shooter.convertDegreestoServoPos(93));
                    }else{
                        shooter.setTurretPos(shooter.convertDegreestoServoPos(-96));
                    }
                    shooter.setHood(0.53);
                    shooter.setTargetVelocity(1300);
                    follower.followPath(toScore1, true);
                })
                .transitionTimed(1.2)

                .state(AutoStates.wait2)
                .onEnter(()->{
                    shooter.setUpperGateOpen(true);
                })
                .transitionTimed(0.1)
                .state(AutoStates.SHOOT2)
                .onEnter(()->{
                    spindexer.setKickerPos(true);
                })
                .transitionTimed(0.4)
                .state(AutoStates.MOVETOINTAKE2)
                .onEnter(()->{
                    spindexer.setKickerPos(false);
                    shooter.setUpperGateOpen(false);
                    follower.followPath(toIntake2, true);
                })

                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.3)
                .state(AutoStates.back2)
                .onEnter(()->{
                    follower.followPath(intake2back, true);
                })

                .transitionTimed(0.5)
                .state(AutoStates.gate2)
                .onEnter(()->{
                    follower.followPath(openGate2, true);
                })

                .transitionTimed(1.2)
                .state(AutoStates.MOVETOSHOOT3)
                .onEnter(()->{
                    if (Posmultiplier==1) {
                        shooter.setTurretPos(shooter.convertDegreestoServoPos(72));
                    }else{
                        shooter.setTurretPos(shooter.convertDegreestoServoPos(-69));
                    }
                    shooter.setHood(0.53);
                    shooter.setTargetVelocity(1330);

                    follower.followPath(toScore2, true);
                })
                .transitionTimed(1.46)

                .state(AutoStates.wait3)
                .onEnter(()->{
                    shooter.setUpperGateOpen(true);
                })
                .transitionTimed(0.1)
                .state(AutoStates.SHOOT3)
                .onEnter(()->{
                    spindexer.setKickerPos(true);
                })

                .transitionTimed(0.4)





                .state(AutoStates.MOVETOINTAKE3)
                .onEnter(()->{
                    spindexer.setKickerPos(false);
                    shooter.setUpperGateOpen(false);
                    PathChain toIntakeGate = follower.pathBuilder()
                            .addPath(new BezierCurve(follower.getPose(), gateintakecontrol,gateintake1))
                            .setLinearHeadingInterpolation(follower.getHeading(), gateintake1.getHeading())
                            .setBrakingStrength(0.4)
                            .build();
                    follower.followPath(toIntakeGate, true);
                })
                .transitionTimed(2.6)

                .state(AutoStates.MOVETOSHOOT4)
                .onEnter(()->{
                    if (Posmultiplier==1) {
                        shooter.setTurretPos(shooter.convertDegreestoServoPos(91));
                    }else{
                        shooter.setTurretPos(shooter.convertDegreestoServoPos(-89));
                    }
                    shooter.setHood(0.53);
                    shooter.setTargetVelocity(1320);
                    PathChain toScoreGate = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), shootPose2nd))
                            .setReversed()

                            .build();
                    follower.followPath(toScoreGate, true);
                })

                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.9)
                .state(AutoStates.wait4)
                .onEnter(()->{
                    intakes.setGoodIntakePower(1);
                    shooter.setUpperGateOpen(true);
                })
                .transitionTimed(0.1)
                .state(AutoStates.SHOOT4)
                .onEnter(()->{
                    spindexer.setKickerPos(true);
                })
                .transitionTimed(0.4)








                .state(AutoStates.MOVETOINTAKE4)
                .onEnter(()->{
                    spindexer.setKickerPos(false);
                    shooter.setUpperGateOpen(false);
                    PathChain toIntakeGate = follower.pathBuilder()
                            .addPath(new BezierCurve(follower.getPose(), gateintakecontrol,gateintake2))
                            .setLinearHeadingInterpolation(follower.getHeading(), gateintake2.getHeading())
                            .setBrakingStrength(0.6)
                            .build();
                    follower.followPath(toIntakeGate, true);
                })
                .transitionTimed(2.6)

                .state(AutoStates.MOVETOSHOOT5)
                .onEnter(()->{
                    if (Posmultiplier==1) {
                        shooter.setTurretPos(shooter.convertDegreestoServoPos(91));
                    }else{
                        shooter.setTurretPos(shooter.convertDegreestoServoPos(-88));
                    }
                    shooter.setHood(0.53);
                    shooter.setTargetVelocity(1320);
                    PathChain toScoreGate = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), shootPose2nd))
                            .setReversed()

                            .build();
                    follower.followPath(toScoreGate, true);
                })

                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.9)
                .state(AutoStates.wait5)
                .onEnter(()->{
                    intakes.setGoodIntakePower(1);
                    shooter.setUpperGateOpen(true);
                })
                .transitionTimed(0.1)
                .state(AutoStates.SHOOT5)
                .onEnter(()->{
                    spindexer.setKickerPos(true);
                })
                .transitionTimed(0.4)









                .state(AutoStates.MOVETOINTAKE5)
                .onEnter(()->{
                    spindexer.setKickerPos(false);
                    shooter.setUpperGateOpen(false);
                    PathChain toIntakeGate = follower.pathBuilder()
                            .addPath(new BezierCurve(follower.getPose(), gateintakecontrol,gateintake3))
                            .setLinearHeadingInterpolation(follower.getHeading(), gateintake3.getHeading())
                            .setBrakingStrength(0.6)
                            .build();
                    follower.followPath(toIntakeGate, true);
                })
                .transitionTimed(2.6)


                .state(AutoStates.MOVETOSHOOT6)
                .onEnter(()->{
                    if (Posmultiplier==1) {
                        shooter.setTurretPos(shooter.convertDegreestoServoPos(91));
                    }else{
                        shooter.setTurretPos(shooter.convertDegreestoServoPos(-88));
                    }
                    shooter.setHood(0.53);
                    shooter.setTargetVelocity(1320);
                    PathChain toScoreGate = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), shootPose2nd))
                            .setReversed()
                            .build();
                    follower.followPath(toScoreGate, true);
                })

                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.9)
                .state(AutoStates.wait6)
                .onEnter(()->{
                    intakes.setGoodIntakePower(1);
                    shooter.setUpperGateOpen(true);
                })
                .transitionTimed(0.1)
                .state(AutoStates.SHOOT6)
                .onEnter(()->{
                    spindexer.setKickerPos(true);
                })
                .transitionTimed(0.4)

                .state(AutoStates.LEAVE)
                .onEnter(()->{
                    PathChain toleave = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), leave))
                            .setLinearHeadingInterpolation(follower.getHeading(), leave.getHeading())
                            .build();
                    follower.followPath(toleave, true);
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
            try {
                if (positionLogger != null){
                    positionLogger.logPose(follower.getPose());
                }
            } catch (IOException ignored) {

            }
        }
        try {
            if (positionLogger != null){
                positionLogger.close();
            }
        } catch (IOException ignored) {

        }
    }
}

