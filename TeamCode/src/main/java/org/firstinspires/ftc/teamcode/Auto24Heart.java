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
@Autonomous(name = "HeartClose24SensorRED", group = "Auto")
public class Auto24Heart extends LinearOpMode {
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
        MOVETOINTAKE1,
        MOVETOSHOOT2, wait2, SHOOT2,
        MOVETOINTAKE2,
        MOVETOSHOOT3, wait3,SHOOT3,
        MOVETOINTAKE3, backupgate1,upgate1,
        MOVETOSHOOT4, wait4,SHOOT4,
        MOVETOINTAKE4,backupgate2, upgate2,
        MOVETOSHOOT5, wait5,SHOOT5,
        MOVETOINTAKE5, waitatIntake1,
        MOVETOSHOOT6, wait6,SHOOT6,
        MOVETOINTAKE6, waitatIntake2,
        MOVETOSHOOT7, wait7,SHOOT7,
        MOVETOINTAKE7, waitatIntake3,
        MOVETOSHOOT8, wait8,SHOOT8,
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


        Pose gateintake1 = new Pose(7, -61*Posmultiplier, Math.toRadians(-110*Posmultiplier));
        Pose gateintakeback1 = new Pose(14, -60*Posmultiplier, Math.toRadians(-110*Posmultiplier));
        Pose gateintake2 = new Pose(5, -61*Posmultiplier, Math.toRadians(-110*Posmultiplier));
        Pose gateintakeback2 = new Pose(14, -60*Posmultiplier, Math.toRadians(-100*Posmultiplier));
        Pose gateintake3 = new Pose(5, -61*Posmultiplier, Math.toRadians(-110*Posmultiplier));
        Pose gateintakeback3 = new Pose(14, -60*Posmultiplier, Math.toRadians(-110*Posmultiplier));
        Pose gateintake4 = new Pose(3, -61*Posmultiplier, Math.toRadians(-110*Posmultiplier));

        Pose gateintakecontrol = new Pose(8, -42*Posmultiplier, Math.toRadians(77*Posmultiplier));

        Pose startPose = new Pose(-44, -55*Posmultiplier, Math.toRadians(6*Posmultiplier));
        Pose startPoseOLD = new Pose(-37, -55.5*Posmultiplier, Math.toRadians(0*Posmultiplier));

        Pose shootPose = new Pose(-32, -16*Posmultiplier, Math.toRadians(-40*Posmultiplier));
        Pose shootPose2nd = new Pose(-17, -17*Posmultiplier, Math.toRadians(-50*Posmultiplier));
        Pose shootPosegoingtolast = new Pose(-20, -20*Posmultiplier, Math.toRadians(-23*Posmultiplier));
        Pose shootPoseleave = new Pose(-30, -16*Posmultiplier, Math.toRadians(-23*Posmultiplier));

        Pose intake1Pose = new Pose(-17, -27*Posmultiplier, Math.toRadians(-80*Posmultiplier));
        Pose intake2Pose = new Pose(10, -26*Posmultiplier, Math.toRadians(-80*Posmultiplier));
        Pose intake3Pose = new Pose(36,-20*Posmultiplier, Math.toRadians(-192*Posmultiplier));
        Pose intake3Poseback = new Pose(34,-40*Posmultiplier, Math.toRadians(-90*Posmultiplier));

        Pose intake1donePose = new Pose(-16, -52*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose intake2donePose = new Pose(6, -56*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose intake3donePose = new Pose(34, -63*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose leave = new Pose(-17, -55*Posmultiplier, Math.toRadians(-90*Posmultiplier));


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

        PathChain toScore1 = follower.pathBuilder()
                .addPath(new BezierLine(intake1donePose, shootPose2nd))
                .setLinearHeadingInterpolation(gateopen.getHeading(), shootPose2nd.getHeading())
                .setBrakingStrength(1)
                .build();


        follower.setStartingPose(startPose);




        StateMachine autoMachine = new StateMachineBuilder() //Autonomia
                .state(AutoStates.MOVETOSHOOT1)
                .onEnter(()->{
                    shooter.setUpperGateOpen(false);
                    spindexer.setLowerGateOpen(true);
                    intakes.setGoodIntakePower(1);
                    follower.followPath(toShoot, true);
                    shooter.setHood(0.36);
                    shooter.setTargetVelocity(1200);
                    if (Posmultiplier==1) {
                        shooter.setTurretPos(shooter.convertDegreestoServoPos(76));
                    }else{
                        shooter.setTurretPos(shooter.convertDegreestoServoPos(-78));
                    }
                })

                .transitionTimed(1.4)
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
                .transition(()->intakes.getGoodBeamBreakOutside() && intakes.getGoodBeamBreakInside() && intakes.getGoodIntakeDetected())


                .state(AutoStates.MOVETOSHOOT2)
                .onEnter(()->{
                    if (Posmultiplier==1) {
                        shooter.setTurretPos(shooter.convertDegreestoServoPos(90));
                    }else{
                        shooter.setTurretPos(shooter.convertDegreestoServoPos(-95));
                    }
                    shooter.setHood(0.36);
                    shooter.setTargetVelocity(1220);
                    follower.followPath(toScore1, true);
                })
                .transitionTimed(1.1)

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
                .transitionTimed(1.4)
                .transition(()->intakes.getGoodBeamBreakOutside() && intakes.getGoodBeamBreakInside() && intakes.getGoodIntakeDetected())
                .state(AutoStates.MOVETOSHOOT3)
                .onEnter(()->{
                    PathChain toScore2 = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), shootPose2nd))
                            .setBrakingStrength(1)
                            .setTangentHeadingInterpolation()
                            .setReversed()
                            .build();
                    if (Posmultiplier==1) {
                        shooter.setTurretPos(shooter.convertDegreestoServoPos(64));
                    }else{
                        shooter.setTurretPos(shooter.convertDegreestoServoPos(-69));
                    }
                    shooter.setHood(0.36);
                    shooter.setTargetVelocity(1250);

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

                .transitionTimed(0.3)





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
                .transition(()->intakes.getGoodBeamBreakOutside() && intakes.getGoodBeamBreakInside() && intakes.getGoodIntakeDetected())
                .transitionTimed(2.5)
                /*
                .state(AutoStates.backupgate1)
                .onEnter(()->{
                    PathChain back = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), gateintakeback1))
                            .setLinearHeadingInterpolation(follower.getHeading(), gateintakeback1.getHeading())
                            .setBrakingStrength(0.4)
                            .build();
                    follower.followPath(back, true);
                })
                .transitionTimed(0.4)
                .state(AutoStates.upgate1)
                .onEnter(()->{
                    PathChain up = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), gateintake1))
                            .setLinearHeadingInterpolation(follower.getHeading(), gateintake1.getHeading())
                            .setBrakingStrength(0.4)
                            .build();
                    follower.followPath(up, true);
                })
                .transitionTimed(0.4)
*/
                .state(AutoStates.MOVETOSHOOT4)
                .onEnter(()->{
                    if (Posmultiplier==1) {
                        shooter.setTurretPos(shooter.convertDegreestoServoPos(78));
                    }else{
                        shooter.setTurretPos(shooter.convertDegreestoServoPos(-82));
                    }
                    shooter.setHood(0.33);
                    shooter.setTargetVelocity(1240);
                    PathChain toScoreGate = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), shootPose2nd))
                            .setReversed()
                            .setBrakingStrength(0.6)
                            .build();
                    follower.followPath(toScoreGate, false);
                })

                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.5)
                .state(AutoStates.wait4)
                .onEnter(()->{
                    follower.pausePathFollowing();
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
                .transition(()->intakes.getGoodBeamBreakOutside() && intakes.getGoodBeamBreakInside() && intakes.getGoodIntakeDetected())
                .transitionTimed(2.5)
                /*
                .state(AutoStates.backupgate2)
                .onEnter(()->{
                    PathChain back = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), gateintakeback2))
                            .setLinearHeadingInterpolation(follower.getHeading(), gateintakeback2.getHeading())
                            .setBrakingStrength(0.4)
                            .build();
                    follower.followPath(back, true);
                })
                .transitionTimed(0.4)
                .state(AutoStates.upgate2)
                .onEnter(()->{
                    PathChain up = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), gateintake2))
                            .setLinearHeadingInterpolation(follower.getHeading(), gateintake2.getHeading())
                            .setBrakingStrength(0.4)
                            .build();
                    follower.followPath(up, true);
                })
                .transitionTimed(0.4)

                 */

                .state(AutoStates.MOVETOSHOOT5)
                .onEnter(()->{
                    if (Posmultiplier==1) {
                        shooter.setTurretPos(shooter.convertDegreestoServoPos(75));
                    }else{
                        shooter.setTurretPos(shooter.convertDegreestoServoPos(-80));
                    }
                    shooter.setHood(0.33);
                    shooter.setTargetVelocity(1250);
                    PathChain toScoreGate = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), shootPose2nd))
                            .setReversed()
                            .setBrakingStrength(0.6)
                            .build();
                    follower.followPath(toScoreGate, false);
                })

                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.5)
                .state(AutoStates.wait5)
                .onEnter(()->{
                    follower.pausePathFollowing();
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
                .transition(()->intakes.getGoodBeamBreakOutside() && intakes.getGoodBeamBreakInside() && intakes.getGoodIntakeDetected())
                .transitionTimed(2.1)
                .state(AutoStates.waitatIntake1)
                .onEnter(()->{
                })
                .transitionTimed(0.3)
                /*
                .state(AutoStates.backupgate3)
                .onEnter(()->{
                    PathChain back = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), gateintakeback3))
                            .setLinearHeadingInterpolation(follower.getHeading(), gateintakeback3.getHeading())
                            .setBrakingStrength(0.4)
                            .build();
                    follower.followPath(back, true);
                })
                                .transitionTimed(0.4)
                .state(AutoStates.upgate3)
                .onEnter(()->{
                    PathChain up = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), gateintake3))
                            .setLinearHeadingInterpolation(follower.getHeading(), gateintake3.getHeading())
                            .setBrakingStrength(0.4)
                            .build();
                    follower.followPath(up, true);
                })
                .transitionTimed(0.4)

                 */

                .state(AutoStates.MOVETOSHOOT6)
                .onEnter(()->{
                    if (Posmultiplier==1) {
                        shooter.setTurretPos(shooter.convertDegreestoServoPos(75));
                    }else{
                        shooter.setTurretPos(shooter.convertDegreestoServoPos(-80));
                    }
                    shooter.setHood(0.33);
                    shooter.setTargetVelocity(1240);
                    PathChain toScoreGate = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), shootPose2nd))
                            .setReversed()
                            .setBrakingStrength(0.6)
                            .build();
                    follower.followPath(toScoreGate, false);
                })

                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.5)
                .state(AutoStates.wait6)
                .onEnter(()->{
                    follower.pausePathFollowing();
                    intakes.setGoodIntakePower(1);
                    shooter.setUpperGateOpen(true);
                })
                .transitionTimed(0.1)
                .state(AutoStates.SHOOT6)
                .onEnter(()->{
                    spindexer.setKickerPos(true);
                })
                .transitionTimed(0.4)
                .state(AutoStates.MOVETOINTAKE6)
                .onEnter(()->{
                    spindexer.setKickerPos(false);
                    shooter.setUpperGateOpen(false);
                    PathChain toIntakeGate = follower.pathBuilder()
                            .addPath(new BezierCurve(follower.getPose(), gateintakecontrol,gateintake4))
                            .setLinearHeadingInterpolation(follower.getHeading(), gateintake4.getHeading())
                            .setBrakingStrength(0.6)
                            .build();
                    follower.followPath(toIntakeGate, true);
                })
                .transition(()->intakes.getGoodBeamBreakOutside() && intakes.getGoodBeamBreakInside() && intakes.getGoodIntakeDetected())
                .transitionTimed(2.5)
                .state(AutoStates.waitatIntake2)
                .onEnter(()->{
                })
                .transitionTimed(0.3)
                /*
                .state(AutoStates.backupgate3)
                .onEnter(()->{
                    PathChain back = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), gateintakeback3))
                            .setLinearHeadingInterpolation(follower.getHeading(), gateintakeback3.getHeading())
                            .setBrakingStrength(0.4)
                            .build();
                    follower.followPath(back, true);
                })
                                .transitionTimed(0.4)
                .state(AutoStates.upgate3)
                .onEnter(()->{
                    PathChain up = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), gateintake3))
                            .setLinearHeadingInterpolation(follower.getHeading(), gateintake3.getHeading())
                            .setBrakingStrength(0.4)
                            .build();
                    follower.followPath(up, true);
                })
                .transitionTimed(0.4)

                 */

                .state(AutoStates.MOVETOSHOOT7)
                .onEnter(()->{
                    if (Posmultiplier==1) {
                        shooter.setTurretPos(shooter.convertDegreestoServoPos(75));
                    }else{
                        shooter.setTurretPos(shooter.convertDegreestoServoPos(-80));
                    }
                    shooter.setHood(0.33);
                    shooter.setTargetVelocity(1240);
                    PathChain toScoreGate = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), shootPose2nd))
                            .setReversed()
                            .setBrakingStrength(0.6)
                            .build();
                    follower.followPath(toScoreGate, false);
                })

                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.5)
                .state(AutoStates.wait7)
                .onEnter(()->{
                    follower.pausePathFollowing();
                    intakes.setGoodIntakePower(1);
                    shooter.setUpperGateOpen(true);
                })
                .transitionTimed(0.1)
                .state(AutoStates.SHOOT7)
                .onEnter(()->{
                    spindexer.setKickerPos(true);
                })
                .transitionTimed(0.3)

                .state(AutoStates.MOVETOSHOOT6)
                .onEnter(()->{
                    if (Posmultiplier==1) {
                        shooter.setTurretPos(shooter.convertDegreestoServoPos(75));
                    }else{
                        shooter.setTurretPos(shooter.convertDegreestoServoPos(-80));
                    }
                    shooter.setHood(0.33);
                    shooter.setTargetVelocity(1240);
                    PathChain toScoreGate = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), shootPose2nd))
                            .setReversed()
                            .setBrakingStrength(0.6)
                            .build();
                    follower.followPath(toScoreGate, false);
                })

                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.5)
                .state(AutoStates.wait6)
                .onEnter(()->{
                    follower.pausePathFollowing();
                    intakes.setGoodIntakePower(1);
                    shooter.setUpperGateOpen(true);
                })
                .transitionTimed(0.1)
                .state(AutoStates.SHOOT6)
                .onEnter(()->{
                    spindexer.setKickerPos(true);
                })
                .transitionTimed(0.4)
                .state(AutoStates.MOVETOINTAKE7)
                .onEnter(()->{
                    spindexer.setKickerPos(false);
                    shooter.setUpperGateOpen(false);
                    PathChain toIntakeGate = follower.pathBuilder()
                            .addPath(new BezierCurve(follower.getPose(), gateintakecontrol,gateintake4))
                            .setLinearHeadingInterpolation(follower.getHeading(), gateintake4.getHeading())
                            .setBrakingStrength(0.6)
                            .build();
                    follower.followPath(toIntakeGate, true);
                })
                .transition(()->intakes.getGoodBeamBreakOutside() && intakes.getGoodBeamBreakInside() && intakes.getGoodIntakeDetected())
                .transitionTimed(2.5)
                .state(AutoStates.waitatIntake3)
                .onEnter(()->{
                })
                .transitionTimed(0.3)
                /*
                .state(AutoStates.backupgate3)
                .onEnter(()->{
                    PathChain back = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), gateintakeback3))
                            .setLinearHeadingInterpolation(follower.getHeading(), gateintakeback3.getHeading())
                            .setBrakingStrength(0.4)
                            .build();
                    follower.followPath(back, true);
                })
                                .transitionTimed(0.4)
                .state(AutoStates.upgate3)
                .onEnter(()->{
                    PathChain up = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), gateintake3))
                            .setLinearHeadingInterpolation(follower.getHeading(), gateintake3.getHeading())
                            .setBrakingStrength(0.4)
                            .build();
                    follower.followPath(up, true);
                })
                .transitionTimed(0.4)

                 */

                .state(AutoStates.MOVETOSHOOT8)
                .onEnter(()->{
                    if (Posmultiplier==1) {
                        shooter.setTurretPos(shooter.convertDegreestoServoPos(75));
                    }else{
                        shooter.setTurretPos(shooter.convertDegreestoServoPos(-80));
                    }
                    shooter.setHood(0.33);
                    shooter.setTargetVelocity(1240);
                    PathChain toScoreGate = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), shootPose2nd))
                            .setReversed()
                            .setBrakingStrength(0.6)
                            .build();
                    follower.followPath(toScoreGate, false);
                })

                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.5)
                .state(AutoStates.wait8)
                .onEnter(()->{
                    follower.pausePathFollowing();
                    intakes.setGoodIntakePower(1);
                    shooter.setUpperGateOpen(true);
                })
                .transitionTimed(0.1)
                .state(AutoStates.SHOOT8)
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

