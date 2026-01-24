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
import org.firstinspires.ftc.teamcode.subsystems.LimelightMotif;
import org.firstinspires.ftc.teamcode.subsystems.Position;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

import java.util.List;

@Configurable
@Autonomous(name = "AutoCloseGate", group = "Auto")
public class AutoStartCloseGate extends LinearOpMode {
    private Follower follower;
    public static int[] shootorder = {0, 1, 2};
    LimelightMotif limelightMotif;
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
        MOVETOINTAKE3, INTAKE3, waitgate1,
        MOVETOSHOOT4, wait4,SHOOT4,
        MOVETOINTAKE4, INTAKE4,waitgate2,
        MOVETOSHOOT5, wait5,SHOOT5,
        MOVETOINTAKE5, INTAKE5, waitgate3,
        MOVETOSHOOT6, wait6,SHOOT6,
        MOVETOINTAKE6, INTAKE6,waitgate4,
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

        limelightMotif = new LimelightMotif(hardwareMap);
        intakes = new Intakes(hardwareMap);
        shooter = new Shooter(hardwareMap);
        spindexer = new Spindexer(hardwareMap);
        follower = createFollower(hardwareMap);

        while (opModeInInit()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();
            follower.update();
            int currpattern = limelightMotif.getMotif();
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


        Pose gateintake = new Pose(20, -57*Posmultiplier, Math.toRadians(70*Posmultiplier));
        Pose gateintakecontrol = new Pose(30, -42*Posmultiplier, Math.toRadians(70*Posmultiplier));
        Pose startPose = new Pose(-40, -55*Posmultiplier, Math.toRadians(-126.3*Posmultiplier));
        Pose shootPose = new Pose(-26, -26*Posmultiplier, Math.toRadians(-40*Posmultiplier));
        Pose shootPose2nd = new Pose(-0, -17*Posmultiplier, Math.toRadians(-50*Posmultiplier));
        Pose shootPose2Control = new Pose(-16, -54.9*Posmultiplier, Math.toRadians(90*Posmultiplier));

        Pose shootPoselast = new Pose(-21, -15*Posmultiplier, Math.toRadians(0*Posmultiplier));
        Pose shootPosegoingtolast = new Pose(-27, -16*Posmultiplier, Math.toRadians(-23*Posmultiplier));
        Pose shootPoseleave = new Pose(-37, -16*Posmultiplier, Math.toRadians(-23*Posmultiplier));

        Pose intake1Pose = new Pose(0, -50*Posmultiplier, Math.toRadians(-30*Posmultiplier));
        Pose intake2Pose = new Pose(30, -50*Posmultiplier, Math.toRadians(-19*Posmultiplier));
        Pose intake3Pose = new Pose(46,-48*Posmultiplier, Math.toRadians(-192*Posmultiplier));
        Pose intake3Posecontrol = new Pose(30,-20*Posmultiplier, Math.toRadians(-90*Posmultiplier));

        Pose intake4Pose = new Pose(33,-64*Posmultiplier, Math.toRadians(-15*Posmultiplier));
        Pose intake4donePose = new Pose(58, -66*Posmultiplier, Math.toRadians(0*Posmultiplier));

        Pose intake1donePose = new Pose(-24, -50*Posmultiplier, Math.toRadians(90*Posmultiplier));
        Pose intake2donePose = new Pose(0, -57*Posmultiplier, Math.toRadians(90*Posmultiplier));
        Pose intake3donePose = new Pose(23, -80*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose leave = new Pose(60, -46*Posmultiplier, Math.toRadians(-90*Posmultiplier));


        PathChain toShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .setBrakingStrength(1.5)
                .build();
        PathChain toIntake1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, intake1Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intake1Pose.getHeading())
                .setBrakingStrength(0.6)
                .build();

        PathChain toIntake2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose2nd, intake2Pose))
                .setTangentHeadingInterpolation()
                .setBrakingStrength(0.6)
                .build();

        PathChain toIntakeGate = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose2nd, gateintakecontrol, gateintake))
                .setTangentHeadingInterpolation()
                .build();

        PathChain toIntake3 = follower.pathBuilder()
                .addPath(new BezierLine(shootPoselast, intake1Pose))
                .setLinearHeadingInterpolation(shootPoselast.getHeading(), intake1Pose.getHeading())
                .build();
        PathChain toIntake4 = follower.pathBuilder()
                .addPath(new BezierLine(shootPosegoingtolast, intake4Pose))
                .setLinearHeadingInterpolation(shootPosegoingtolast.getHeading(), intake4Pose.getHeading())
                .build();

        PathChain toIntake1fin = follower.pathBuilder()
                .addPath(new BezierLine(intake3Pose, intake3donePose))
                .setLinearHeadingInterpolation(intake3Pose.getHeading(), intake3donePose.getHeading())
                .build();

        PathChain toIntake2fin = follower.pathBuilder()
                .addPath(new BezierLine(intake2Pose, intake2donePose))
                .setLinearHeadingInterpolation(intake2Pose.getHeading(), intake2donePose.getHeading())
                .build();

        PathChain toIntake2back= follower.pathBuilder()
                .addPath(new BezierLine(intake2donePose, intake2Pose))
                .setLinearHeadingInterpolation(intake2donePose.getHeading(), intake2Pose.getHeading())
                .build();

        PathChain toIntake3fin = follower.pathBuilder()
                .addPath(new BezierLine(intake1Pose, intake1donePose))
                .setLinearHeadingInterpolation(intake1Pose.getHeading(), intake1donePose.getHeading())
                .build();
        PathChain toIntake4fin = follower.pathBuilder()
                .addPath(new BezierLine(intake4Pose, intake4donePose))
                .setLinearHeadingInterpolation(intake4Pose.getHeading(), intake4donePose.getHeading())
                .build();

        PathChain toScore1 = follower.pathBuilder()
                .addPath(new BezierLine(intake1Pose, shootPose2nd))
                .setLinearHeadingInterpolation(intake1Pose.getHeading(), shootPose2nd.getHeading())
                .build();

        PathChain toScore2 = follower.pathBuilder()
                .addPath(new BezierLine(intake2Pose, shootPose2nd))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        PathChain toScore3 = follower.pathBuilder()
                .addPath(new BezierLine(intake1donePose, shootPosegoingtolast))
                .setLinearHeadingInterpolation(intake1donePose.getHeading(), shootPosegoingtolast.getHeading())
                .build();
        PathChain toScoreGate = follower.pathBuilder()
                .addPath(new BezierCurve(gateintake, gateintakecontrol, shootPose2nd))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        PathChain park = follower.pathBuilder()
                .addPath(new BezierLine(shootPoselast, leave))
                .setLinearHeadingInterpolation(shootPoselast.getHeading(), leave.getHeading())
                .build();


        follower.setStartingPose(startPose);


        StateMachine stateMachine = new StateMachineBuilder()
                .state(States.Intake)
                .loop(()->{
                    shooter.setUpperGateOpen(false);
                    spindexer.setLowerGateOpen(rapidFire);
                    spindexer.setKickerPos(false);

                    if (rapidFire) {
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot1);

                        intakes.setGoodIntakePower(1);
                        intakes.setBadIntakePower(-0.1);
                    }else{
                        spindexer.setPosition(Spindexer.SpindexerPosition.Intake1);
                        intakes.setGoodIntakePower(0.2);
                        intakes.setBadIntakePower(1);
                    }
                })
                .transition(()->shooterButton && rapidFire, States.OpenUpperGate)
                .transition(()->shooterButton && !rapidFire, States.Kick1)
                .transition(()->intakes.getBadIntakeDetected() && !rapidFire, States.Wait1)

                .state(States.Wait1)
                .onEnter(()->{
                    spindexer.setPosition(Spindexer.SpindexerPosition.Intake2);
                })
                .transitionTimed(shootWaitTime, States.Increment2)
                .transition(()->shooterButton, States.Kick1)

                .state(States.Increment2)
                .onEnter(()->{
                    spindexer.setPosition(Spindexer.SpindexerPosition.Intake2);
                })
                .transition(()->intakes.getBadIntakeDetected(), States.Wait2)
                .transition(()->shooterButton, States.Kick1)

                .state(States.Wait2)
                .onEnter(()->{
                    spindexer.setPosition(Spindexer.SpindexerPosition.Intake3);
                })
                .transitionTimed(shootWaitTime, States.Increment3)
                .transition(()->shooterButton, States.Kick1)

                .state(States.Increment3)
                .onEnter(()->{
                    spindexer.setPosition(Spindexer.SpindexerPosition.Intake3);
                })
                .transition(()->intakes.getBadIntakeDetected(), States.WaitForShoot)
                .transition(()->shooterButton, States.Kick1)

                .state(States.WaitForShoot)
                .loop(()->{
                    if (shootorder[0]==0){
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot1);
                    }else if (shootorder[0]==1){
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot2);
                    }else{
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot3);
                    }
                    intakes.setBadIntakePower(0.3);
                    intakes.setGoodIntakePower(1);

                    shooter.setUpperGateOpen(true);
                })
                .transition(()->shooterButton, States.Kick1)

                .state(States.Kick1)
                .onEnter(()->{
                    shooterButton=false;
                    if (shootorder[0]==0){
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot1);
                    }else if (shootorder[0]==1){
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot2);
                    }else{
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot3);
                    }
                    spindexer.setLowerGateOpen(true);
                    shooter.setUpperGateOpen(true);
                    spindexer.setKickerPos(true);
                })
                .transitionTimed(shootWaitTime, States.ShootSpin1)

                .state(States.ShootSpin1)
                .onEnter(()->{
                    if (shootorder[1]==0){
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot1);
                    }else if (shootorder[1]==1){
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot2);
                    }else{
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot3);
                    }
                    spindexer.setKickerPos(false);
                })
                .transitionTimed(shootWaitTime, States.Kick2)

                .state(States.Kick2)
                .onEnter(()->{
                    spindexer.setKickerPos(true);
                })
                .transitionTimed(shootWaitTime, States.ShootSpin2)

                .state(States.ShootSpin2)
                .onEnter(()->{
                    if (shootorder[2]==0){
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot1);
                    }else if (shootorder[2]==1){
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot2);
                    }else{
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot3);
                    }
                    spindexer.setKickerPos(false);
                })
                .transitionTimed(shootWaitTime, States.Kick3)

                .state(States.Kick3)
                .onEnter(()->{
                    spindexer.setKickerPos(true);
                })
                .transitionTimed(shootWaitTime, States.Intake)

                .state(States.OpenUpperGate)
                .onEnter(()->{
                    shooterButton=false;
                    spindexer.setLowerGateOpen(true);
                    shooter.setUpperGateOpen(true);
                })
                .transitionTimed(0.2, States.Shoot)

                .state(States.Shoot)
                .onEnter(()->{
                    spindexer.setKickerPos(true);
                })
                .transitionTimed(1.2, States.Intake)
                .build();


        StateMachine autoMachine = new StateMachineBuilder() //Autonomia
                .state(AutoStates.MOVETOSHOOT1)
                .onEnter(()->{
                    intakes.setGoodIntakePower(1);
                    follower.followPath(toShoot, true);
                    shooter.setHood(0.49);
                    shooter.setTargetVelocity(1300);
                    shooter.setTurretPos(shooter.convertDegreestoServoPos(105*Posmultiplier));
                })

                .transitionTimed(2)
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
                .transitionTimed(0.6)

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
                    shooter.setHood(0.52);
                    shooter.setTargetVelocity(1400);
                    follower.followPath(toScore1, true);
                })
                .transition(()->!follower.isBusy())
                .transitionTimed(2)

                .state(AutoStates.wait2)
                .onEnter(()->{
                    shooter.setUpperGateOpen(true);
                })
                .transitionTimed(0.2)
                .state(AutoStates.SHOOT2)
                .onEnter(()->{
                    spindexer.setKickerPos(true);
                })
                .transitionTimed(0.6)
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
                    follower.followPath(toScore2, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.3)

                .state(AutoStates.wait3)
                .onEnter(()->{
                    shooter.setUpperGateOpen(true);
                })
                .transitionTimed(0.2)
                .state(AutoStates.SHOOT3)
                .onEnter(()->{
                    spindexer.setKickerPos(true);
                    shooterButton=true;
                })

                .transitionTimed(0.5)

                .state(AutoStates.MOVETOINTAKE3)
                .onEnter(()->{
                    spindexer.setKickerPos(false);
                    shooter.setUpperGateOpen(false);
                    follower.followPath(toIntakeGate, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(2)
                .state(AutoStates.waitgate1)
                .onEnter(()->{

                })
                .transitionTimed(1.6)
                .state(AutoStates.MOVETOSHOOT4)
                .onEnter(()->{
                    follower.followPath(toScoreGate, true);
                })

                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.7)
                .state(AutoStates.wait4)
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
                    spindexer.setKickerPos(false);
                    shooter.setUpperGateOpen(false);
                    follower.followPath(toIntakeGate, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(2.1)
                .state(AutoStates.waitgate2)
                .onEnter(()->{
                })
                .transitionTimed(1)
                .state(AutoStates.MOVETOSHOOT5)
                .onEnter(()->{
                    follower.followPath(toScoreGate, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(2)
                .state(AutoStates.wait5)
                .onEnter(()->{
                    shooter.setUpperGateOpen(true);
                })
                .transitionTimed(0.3)
                .state(AutoStates.SHOOT5)
                .onEnter(()->{
                    spindexer.setKickerPos(true);
                })
                .transitionTimed(0.5)





                .state(AutoStates.MOVETOINTAKE5)
                .onEnter(()->{
                    spindexer.setKickerPos(false);
                    shooter.setUpperGateOpen(false);
                    follower.followPath(toIntakeGate, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(2.1)
                .state(AutoStates.waitgate3)
                .onEnter(()->{
                })
                .transitionTimed(1)
                .state(AutoStates.MOVETOSHOOT6)
                .onEnter(()->{
                    follower.followPath(toScoreGate, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(2)
                .state(AutoStates.wait6)
                .onEnter(()->{
                    shooter.setUpperGateOpen(true);
                })
                .transitionTimed(0.3)
                .state(AutoStates.SHOOT6)
                .onEnter(()->{
                    spindexer.setKickerPos(true);
                })
                .transitionTimed(0.5)



                .state(AutoStates.MOVETOINTAKE6)
                .onEnter(()->{
                    spindexer.setKickerPos(false);
                    shooter.setUpperGateOpen(false);
                    follower.followPath(toIntakeGate, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(2.1)
                .state(AutoStates.waitgate4)
                .onEnter(()->{
                })
                .transitionTimed(1.1)
                .state(AutoStates.MOVETOSHOOT7)
                .onEnter(()->{
                    follower.followPath(toScoreGate, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(2)
                .state(AutoStates.wait7)
                .onEnter(()->{
                    shooter.setUpperGateOpen(true);
                })
                .transitionTimed(0.3)
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
            telemetry.addData("State: ", stateMachine.getState());
            telemetry.addData("State auto: ", autoMachine.getState());
            telemetry.addData("Shooter Target", shooter.getTargetVelo());
            telemetry.addData("Shooter Velocity", shooter.getCurrentVelocity());
            telemetry.addData("Spindexer kick", spindexer.is_kick);
            telemetry.addData("Pose: ", follower.getPose());
            telemetry.update();
        }
    }
}

