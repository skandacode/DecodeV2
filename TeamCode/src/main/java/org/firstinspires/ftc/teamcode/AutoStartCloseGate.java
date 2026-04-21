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
import org.firstinspires.ftc.teamcode.subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.subsystems.Position;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

import java.io.IOException;
import java.util.Arrays;
import java.util.List;

@Configurable
@Autonomous(name = "AutoCloseGate", group = "Auto")
public class AutoStartCloseGate extends LinearOpMode {
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
        MOVETOINTAKE3, backupgate1,waitgate1, upgate,
        MOVETOSHOOT4, wait4,SHOOT4,
        MOVETOINTAKE4,
        MOVETOSHOOT5, wait5,SHOOT5,
        MOVETOINTAKE5, backupgate2,waitgate2, upgate1,
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


        Pose gateintake = new Pose(2, -57*Posmultiplier, Math.toRadians(-90*Posmultiplier));

        Pose gateintakeback = new Pose(16, -60*Posmultiplier, Math.toRadians(-130*Posmultiplier));
        Pose gateintakebackin = new Pose(12, -60*Posmultiplier, Math.toRadians(-130*Posmultiplier));

        Pose gateintakecontrol = new Pose(8, -42*Posmultiplier, Math.toRadians(77*Posmultiplier));
        Pose startPose = new Pose(-44, -55*Posmultiplier, Math.toRadians(0*Posmultiplier));

        Pose shootPose = new Pose(-32, -16*Posmultiplier, Math.toRadians(-40*Posmultiplier));
        Pose shootPose2nd = new Pose(-17, -17*Posmultiplier, Math.toRadians(-50*Posmultiplier));
        Pose shootPosegoingtolast = new Pose(-20, -20*Posmultiplier, Math.toRadians(-23*Posmultiplier));
        Pose shootPoseleave = new Pose(-30, -16*Posmultiplier, Math.toRadians(-23*Posmultiplier));

        Pose intake1Pose = new Pose(-15, -27*Posmultiplier, Math.toRadians(-80*Posmultiplier));
        Pose intake2Pose = new Pose(13, -29*Posmultiplier, Math.toRadians(-80*Posmultiplier));
        Pose intake3Pose = new Pose(36,-20*Posmultiplier, Math.toRadians(-192*Posmultiplier));
        Pose intake3Poseback = new Pose(34,-40*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose intake4Pose = new Pose(28,-54*Posmultiplier, Math.toRadians(0*Posmultiplier));
        Pose intake4donePose = new Pose(52, -63*Posmultiplier, Math.toRadians(0*Posmultiplier));
        Pose intake4Poseback = new Pose(48, -63*Posmultiplier, Math.toRadians(0*Posmultiplier));
        Pose intake4donePosemore = new Pose(58, -63*Posmultiplier, Math.toRadians(0*Posmultiplier));

        Pose intake1donePose = new Pose(-14, -52*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose intake2donePose = new Pose(10, -63*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose intake3donePose = new Pose(34, -63*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose leave = new Pose(-20, -55*Posmultiplier, Math.toRadians(-90*Posmultiplier));


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
                .setBrakingStrength(0.6)
                .build();

        PathChain toIntakeGate = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose2nd, gateintakecontrol, gateintake))
                .setLinearHeadingInterpolation(shootPose2nd.getHeading(), gateintake.getHeading())
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
                .addPath(new BezierLine(intake4Poseback, intake4donePosemore))
                .setConstantHeadingInterpolation(intake4donePosemore.getHeading())
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
                .addPath(new BezierLine(intake3donePose, shootPose2nd))
                .setBrakingStrength(4)
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
        PathChain toScore4 = follower.pathBuilder()
                .addPath(new BezierLine(intake4donePose, shootPoseleave))
                .setBrakingStrength(20)
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
                    shooter.setHood(0.52);
                    shooter.setTargetVelocity(1340);
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
                .transitionTimed(0.55)

                .state(AutoStates.MOVETOINTAKE1)
                .onEnter(()->{
                    shooter.setUpperGateOpen(false);
                    spindexer.setKickerPos(false);
                    follower.followPath(toIntake1, true);
                })

                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.2)

                .state(AutoStates.MOVETOSHOOT2)
                .onEnter(()->{
                    if (Posmultiplier==1) {
                        shooter.setTurretPos(shooter.convertDegreestoServoPos(93));
                    }else{
                        shooter.setTurretPos(shooter.convertDegreestoServoPos(-96));
                    }
                    shooter.setHood(0.54);
                    shooter.setTargetVelocity(1320);
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

                .state(AutoStates.MOVETOSHOOT3)
                .onEnter(()->{
                    if (Posmultiplier==1) {
                        shooter.setTurretPos(shooter.convertDegreestoServoPos(72));
                    }else{
                        shooter.setTurretPos(shooter.convertDegreestoServoPos(-74));
                    }                    shooter.setHood(0.54);
                    shooter.setTargetVelocity(1370);

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
                    shooterButton=true;
                })

                .transitionTimed(0.6)

                .state(AutoStates.MOVETOINTAKE3)
                .onEnter(()->{
                    spindexer.setKickerPos(false);
                    shooter.setUpperGateOpen(false);
                    follower.followPath(toIntakeGate, true);
                })
                .transitionTimed(1.2)
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
                .transitionTimed(0.5)
                .state(AutoStates.waitgate1)
                .onEnter(()->{

                })
                .transitionTimed(0.5)
                .state(AutoStates.upgate)
                .onEnter(()->{

                    PathChain tointakeinGate = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), gateintakebackin))
                            .setLinearHeadingInterpolation(follower.getHeading(), gateintakebackin.getHeading())
                            .build();
                    follower.followPath(tointakeinGate, true);


                })
                .transitionTimed(0.5)
                .state(AutoStates.MOVETOSHOOT4)
                .onEnter(()->{
                    intakes.setFrontIntakePower(0);
                    if (Posmultiplier==1) {
                        shooter.setTurretPos(shooter.convertDegreestoServoPos(91));
                    }else{
                        shooter.setTurretPos(shooter.convertDegreestoServoPos(-95));
                    }
                    shooter.setTargetVelocity(1350);
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
                .transitionTimed(0.6)








                .state(AutoStates.MOVETOINTAKE4)
                .onEnter(()->{
                    spindexer.setKickerPos(false);
                    shooter.setUpperGateOpen(false);
                    follower.followPath(toIntake3, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.8)

                .state(AutoStates.MOVETOSHOOT5)
                .onEnter(()->{
                    if (Posmultiplier==1) {
                        shooter.setTurretPos(shooter.convertDegreestoServoPos(93));
                    }else{
                        shooter.setTurretPos(shooter.convertDegreestoServoPos(-97));
                    }
                    shooter.setHood(0.6);
                    shooter.setTargetVelocity(1380);
                    follower.followPath(toScore3, true);
                    intakes.setFrontIntakePower(0);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(2)
                .state(AutoStates.wait5)
                .onEnter(()->{
                    shooter.setUpperGateOpen(true);
                    intakes.setGoodIntakePower(1);

                })
                .transitionTimed(0.1)
                .state(AutoStates.SHOOT5)
                .onEnter(()->{
                    spindexer.setKickerPos(true);
                })
                .transitionTimed(0.6)








                .state(AutoStates.MOVETOINTAKE5)
                .onEnter(()->{
                    spindexer.setKickerPos(false);
                    shooter.setUpperGateOpen(false);
                    PathChain toIntakeGatee = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), gateintake))
                            .setLinearHeadingInterpolation(follower.getHeading(), gateintake.getHeading())
                            .build();
                    follower.followPath(toIntakeGatee, true);
                })
                .transitionTimed(1.3)

                .state(AutoStates.waitgate2)
                .onEnter(()->{

                    PathChain toIntakeBackoutGate = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), gateintakeback))
                            .setLinearHeadingInterpolation(follower.getHeading(), gateintakeback.getHeading())
                            .build();
                    follower.followPath(toIntakeBackoutGate, true);

                })
                .transitionTimed(1)
                .state(AutoStates.upgate1)
                .onEnter(()->{

                    PathChain tointakeinGate = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), gateintakebackin))
                            .setLinearHeadingInterpolation(follower.getHeading(), gateintakebackin.getHeading())
                            .build();
                    follower.followPath(tointakeinGate, true);


                })
                .transitionTimed(0.5)
                .state(AutoStates.MOVETOSHOOT6)
                .onEnter(()->{
                    intakes.setFrontIntakePower(0);
                    if (Posmultiplier==1) {
                        shooter.setTurretPos(shooter.convertDegreestoServoPos(123));
                    }else{
                        shooter.setTurretPos(shooter.convertDegreestoServoPos(-125));
                    }
                    shooter.setHood(0.52);
                    shooter.setTargetVelocity(1340);
                    PathChain toScorenobackgate = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), shootPosegoingtolast))
                            .setLinearHeadingInterpolation(follower.getHeading(), shootPosegoingtolast.getHeading())
                            .build();
                    follower.followPath(toScorenobackgate, true);
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
                .transitionTimed(0.6)


                .state(AutoStates.MOVETOINTAKE6)
                .onEnter(()->{
                    spindexer.setKickerPos(false);
                    shooter.setUpperGateOpen(false);
                    follower.followPath(toIntake4, true);
                })
                .transitionTimed(1.8)
                /*
                .state(AutoStates.INTAKE6BACK)
                .onEnter(()->{
                    follower.followPath(toIntake4back, true);
                })
                .transitionTimed(0.12)
                .state(AutoStates.INTAKE6IN)
                .onEnter(()->{
                    follower.followPath(toIntake4in, true);
                })
                .transitionTimed(0.4)
*/
                .state(AutoStates.MOVETOSHOOT7)
                .onEnter(()->{
                    if (Posmultiplier==1) {
                        shooter.setTurretPos(shooter.convertDegreestoServoPos(86));
                    }else{
                        shooter.setTurretPos(shooter.convertDegreestoServoPos(-100));
                    }
                    shooter.setTargetVelocity(1270);
                    shooter.setHood(0.53);
                    follower.followPath(toScore4, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(2.3)
                .state(AutoStates.wait7)
                .onEnter(()->{
                    shooter.setUpperGateOpen(true);
                })
                .transitionTimed(0.1)
                .state(AutoStates.SHOOT7)
                .onEnter(()->{
                    spindexer.setKickerPos(true);

                })
                .transitionTimed(0.3)
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

