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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
import java.util.List;
@Disabled
@Configurable
@Autonomous(name = "Auto24", group = "Auto")
public class Auto24Close extends LinearOpMode {
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
        MOVETOINTAKE3, backupgate1,waitgate1,
        MOVETOSHOOT4, wait4,SHOOT4,
        MOVETOINTAKE4,
        MOVETOSHOOT5, wait5,SHOOT5,
        MOVETOINTAKE5, backupgate2,waitgate2,
        MOVETOSHOOT6, wait6,SHOOT6,
        MOVETOINTAKE6,
        MOVETOSHOOT7, wait7,SHOOT7,
        MOVETOINTAKE7,
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
            spindexer.setPosition(Spindexer.SpindexerPosition.Shoot1);
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


        Pose gateintake = new Pose(19, -57*Posmultiplier, Math.toRadians(-90*Posmultiplier));

        Pose gateintakeback = new Pose(25, -60*Posmultiplier, Math.toRadians(-130*Posmultiplier));
        Pose gateintakebackin = new Pose(24, -60*Posmultiplier, Math.toRadians(-130*Posmultiplier));

        Pose gateintakecontrol = new Pose(20, -42*Posmultiplier, Math.toRadians(77*Posmultiplier));
        Pose startPose = new Pose(-52, -58*Posmultiplier, Math.toRadians(0*Posmultiplier));

        Pose shootPose = new Pose(-40, -39*Posmultiplier, Math.toRadians(-0*Posmultiplier));
        Pose shootPose2nd = new Pose(-0, -17*Posmultiplier, Math.toRadians(-50*Posmultiplier));
        Pose shootPosegoingtolast = new Pose(-4, -20*Posmultiplier, Math.toRadians(-23*Posmultiplier));
        Pose shootPoseleave = new Pose(-30, -16*Posmultiplier, Math.toRadians(-23*Posmultiplier));

        Pose intake1Pose = new Pose(-35, -50*Posmultiplier, Math.toRadians(0*Posmultiplier));
        Pose intake2Pose = new Pose(35, -50*Posmultiplier, Math.toRadians(0*Posmultiplier));
        Pose intake3Pose = new Pose(48,-20*Posmultiplier, Math.toRadians(-192*Posmultiplier));
        Pose intake3Poseback = new Pose(46,-40*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose intake4Pose = new Pose(40,-54*Posmultiplier, Math.toRadians(0*Posmultiplier));
        Pose intake4donePose = new Pose(64, -58*Posmultiplier, Math.toRadians(0*Posmultiplier));
        Pose intake4Poseback = new Pose(60, -58*Posmultiplier, Math.toRadians(0*Posmultiplier));
        Pose intake4donePosemore = new Pose(70, -58*Posmultiplier, Math.toRadians(0*Posmultiplier));

        Pose intake1donePose = new Pose(-18, -50*Posmultiplier, Math.toRadians(0*Posmultiplier));
        Pose intake2donePose = new Pose(16, -51*Posmultiplier, Math.toRadians(0*Posmultiplier));
        Pose intake3donePose = new Pose(48, -63*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose leave = new Pose(60, -46*Posmultiplier, Math.toRadians(-90*Posmultiplier));


        PathChain toShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .setBrakingStrength(1.5)
                .build();
        PathChain toIntake1 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, intake1Pose, intake1donePose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intake1donePose.getHeading())
                .build();

        PathChain toIntake2 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, intake2Pose, intake2donePose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intake2donePose.getHeading())
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
                .addPath(new BezierLine(intake1donePose, shootPose))
                .setLinearHeadingInterpolation(intake1donePose.getHeading(), shootPose.getHeading())
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
                    shooter.setHood(0.47);
                    shooter.setTargetVelocity(1240);
                    shooter.setTurretPos(shooter.convertDegreestoServoPos(115*Posmultiplier));
                })

                .transitionTimed(1)
                .state(AutoStates.wait1)
                .onEnter(()->{
                    shooter.setUpperGateOpen(true);

                })
                .transitionTimed(0.1)
                .state(AutoStates.SHOOT1)
                .onEnter(()->{
                    spindexer.setKickerPos(true);
                })
                .transitionTimed(0.5)

                .state(AutoStates.MOVETOINTAKE1)
                .onEnter(()->{
                    shooter.setUpperGateOpen(false);
                    spindexer.setKickerPos(false);
                    follower.followPath(toIntake1, true);
                })

                .transition(()->follower.atParametricEnd())
                .transitionTimed(0.7)

                .state(AutoStates.MOVETOSHOOT2)
                .onEnter(()->{
                    shooter.setTurretPos(shooter.convertDegreestoServoPos(115*Posmultiplier));
                    shooter.setHood(0.46);
                    shooter.setTargetVelocity(1300);
                    follower.followPath(toScore1, true);
                })
                .transition(()->!follower.isBusy())
                .transitionTimed(1.8)

                .state(AutoStates.wait2)
                .onEnter(()->{
                    shooter.setUpperGateOpen(true);
                })
                .transitionTimed(0.3)
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
                    shooter.setTurretPos(shooter.convertDegreestoServoPos(72*Posmultiplier));
                    shooter.setHood(0.56);
                    shooter.setTargetVelocity(1380);

                    follower.followPath(toScore2, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.36)

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

                .transitionTimed(0.4)

                .state(AutoStates.MOVETOINTAKE3)
                .onEnter(()->{
                    spindexer.setKickerPos(false);
                    shooter.setUpperGateOpen(false);
                    follower.followPath(toIntakeGate, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.1)
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
                .transitionTimed(0.7)
                .state(AutoStates.MOVETOSHOOT4)
                .onEnter(()->{
                    shooter.setTurretPos(shooter.convertDegreestoServoPos(91*Posmultiplier));
                    shooter.setTargetVelocity(1400);
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
                .transitionTimed(0.5)








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
                    shooter.setTurretPos(shooter.convertDegreestoServoPos(93*Posmultiplier));
                    shooter.setHood(0.6);
                    shooter.setTargetVelocity(1380);
                    follower.followPath(toScore3, true);
                    intakes.setBadIntakePower(-0.5);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(2)
                .state(AutoStates.wait5)
                .onEnter(()->{
                    shooter.setUpperGateOpen(true);
                    intakes.setBadIntakePower(0);

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
                    PathChain toIntakeGatee = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), gateintake))
                            .setLinearHeadingInterpolation(follower.getHeading(), gateintake.getHeading())
                            .build();
                    follower.followPath(toIntakeGatee, true);
                })
                .transitionTimed(1)

                .state(AutoStates.waitgate2)
                .onEnter(()->{

                    PathChain toIntakeBackoutGate = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), gateintakeback))
                            .setLinearHeadingInterpolation(follower.getHeading(), gateintakeback.getHeading())
                            .build();
                    follower.followPath(toIntakeBackoutGate, true);

                })
                .transitionTimed(1.2)
                .state(AutoStates.MOVETOSHOOT6)
                .onEnter(()->{
                    shooter.setTurretPos(shooter.convertDegreestoServoPos(123*Posmultiplier));
                    shooter.setTargetVelocity(1360);
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
                .transitionTimed(0.5)


                .state(AutoStates.MOVETOINTAKE6)
                .onEnter(()->{
                    spindexer.setKickerPos(false);
                    shooter.setUpperGateOpen(false);
                    follower.followPath(toIntake4, true);
                })
                .transitionTimed(1.8)
                .state(AutoStates.MOVETOSHOOT7)
                .onEnter(()->{
                    intakes.setBadIntakePower(-0.3);
                    shooter.setTurretPos(shooter.convertDegreestoServoPos(86*Posmultiplier));
                    shooter.setTargetVelocity(1300);
                    shooter.setHood(0.57);
                    follower.followPath(toScore4, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(2)
                .state(AutoStates.wait7)
                .onEnter(()->{
                    shooter.setUpperGateOpen(true);
                })
                .transitionTimed(0.1)
                .state(AutoStates.SHOOT7)
                .onEnter(()->{
                    spindexer.setKickerPos(true);
                })
                .transitionTimed(0.4)

                .state(AutoStates.MOVETOINTAKE7)
                .onEnter(()->{
                    spindexer.setKickerPos(false);
                    shooter.setUpperGateOpen(false);
                    follower.followPath(toIntake4, true);
                })
                .transitionTimed(1.8)
                .state(AutoStates.MOVETOSHOOT8)
                .onEnter(()->{
                    intakes.setBadIntakePower(-0.3);
                    shooter.setTurretPos(shooter.convertDegreestoServoPos(86*Posmultiplier));
                    shooter.setTargetVelocity(1300);
                    shooter.setHood(0.57);
                    follower.followPath(toScore4, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(2)
                .state(AutoStates.wait8)
                .onEnter(()->{
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

