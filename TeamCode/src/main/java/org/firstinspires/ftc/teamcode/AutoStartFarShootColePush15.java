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

import org.firstinspires.ftc.teamcode.subsystems.Position;
import org.firstinspires.ftc.teamcode.subsystems.Intakes;
import org.firstinspires.ftc.teamcode.subsystems.LimelightMotif;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

import java.util.Arrays;
import java.util.List;

@Configurable
@Autonomous(name = "AutoFarIndex15", group = "Auto")
public class AutoStartFarShootColePush15 extends LinearOpMode {
    private Follower follower;
    public static int[] shootorder = {0, 1, 2};
    public boolean shooterButtonAll = false;
    LimelightMotif limelightMotif;
    Intakes intakes;
    String colorAlliance = "BLUE";
    int Posmultiplier = 1;
    Shooter shooter;
    Spindexer spindexer;
    public int pattern = 1;
    public boolean shooterButton = false;
    public double shootWaitTime = 0.25;
    public static boolean rapidFire = true;



    public static Shooter.Goal shooterTarget = Shooter.Goal.BLUE;


    public enum AutoStates {
        PUSH,
        MOVETOSHOOT1, wait1, SHOOT1,
        MOVETOINTAKE1, INTAKE1,
        MOVETOSHOOT2, wait2, SHOOT2,
        MOVETOINTAKE2, INTAKE2, INTAKE2BACK, BACK, GATE, waitgate,
        MOVETOSHOOT3, wait3,SHOOT3,
        MOVETOINTAKE3, INTAKE3,
        MOVETOSHOOT4, wait4,SHOOT4,
        MOVETOINTAKE4, INTAKE4, INTAKE4BACK, INTAKE4BACKIN,
        MOVETOSHOOT5, wait5,SHOOT5,
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
        ShootSpin2,
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
        }

        waitForStart();


        Pose opengate = new Pose(-12, -55*Posmultiplier, Math.toRadians(90*Posmultiplier));
        Pose opengateback = new Pose(-7, -38*Posmultiplier, Math.toRadians(90*Posmultiplier));
        Pose startPose = new Pose(60, -15*Posmultiplier, Math.toRadians(0*Posmultiplier));
        Pose pushPose = new Pose(58, -26*Posmultiplier, Math.toRadians(0*Posmultiplier));
        Pose shootPose = new Pose(-18, -22*Posmultiplier, Math.toRadians(0*Posmultiplier));
        Pose shootPose2 = new Pose(-16, -29*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose shootPoselast = new Pose(-21, -15*Posmultiplier, Math.toRadians(90*Posmultiplier));
        Pose shootPoseleave = new Pose(-37, -16*Posmultiplier, Math.toRadians(-23*Posmultiplier));

        Pose intake1Pose = new Pose(-24, -21*Posmultiplier, Math.toRadians(90*Posmultiplier));
        Pose intake2Pose = new Pose(0, -25*Posmultiplier, Math.toRadians(90*Posmultiplier));
        Pose intake3Pose = new Pose(23,-42*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose intake3Posecontrol = new Pose(30,-20*Posmultiplier, Math.toRadians(-90*Posmultiplier));

        Pose intake4Pose = new Pose(33,-64*Posmultiplier, Math.toRadians(-15*Posmultiplier));
        Pose intake4donePose = new Pose(58, -66*Posmultiplier, Math.toRadians(0*Posmultiplier));

        Pose intake1donePose = new Pose(-24, -46*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose intake2donePose = new Pose(0, -55*Posmultiplier, Math.toRadians(90*Posmultiplier));
        Pose intake3donePose = new Pose(23, -80*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose leave = new Pose(60, -46*Posmultiplier, Math.toRadians(-90*Posmultiplier));


        PathChain toShoot = follower.pathBuilder()
                .addPath(new BezierLine(pushPose, shootPose))
                .setLinearHeadingInterpolation(pushPose.getHeading(), shootPose.getHeading())
                .setBrakingStrength(1.5)
                .build();
        PathChain toPush = follower.pathBuilder()
                .addPath(new BezierLine(startPose, pushPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), pushPose.getHeading())
                .setBrakingStrength(1.5)
                .build();

        PathChain toIntake1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, intake3Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intake3Pose.getHeading())
                .build();

        PathChain toIntake2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, intake2Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intake2Pose.getHeading())
                .setBrakingStrength(0.9)
                .build();

        PathChain opengatein = follower.pathBuilder()
                .addPath(new BezierLine(opengateback, opengate))
                .setLinearHeadingInterpolation(opengateback.getHeading(), opengate.getHeading())
                .build();

        PathChain intake2ToGate= follower.pathBuilder()
                .addPath(new BezierLine(intake2donePose, opengateback))
                .setLinearHeadingInterpolation(intake2donePose.getHeading(), opengateback.getHeading())
                .build();

        PathChain toIntake3 = follower.pathBuilder()
                .addPath(new BezierLine(shootPoselast, intake4Pose))
                .setLinearHeadingInterpolation(shootPoselast.getHeading(), intake4Pose.getHeading())
                .setBrakingStrength(0.8)
                .build();
        PathChain toIntake4 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, intake4Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intake4Pose.getHeading())
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
                .addPath(new BezierCurve(intake3donePose, intake3Posecontrol, shootPose))
                .setLinearHeadingInterpolation(intake3donePose.getHeading(), shootPose.getHeading())
                .setBrakingStrength(0.8)
                .build();

        PathChain toScore2 = follower.pathBuilder()
                .addPath(new BezierCurve(opengate, opengateback,shootPose2))
                .setLinearHeadingInterpolation(opengate.getHeading(), shootPose2.getHeading())
                .setBrakingStrength(0.9)
                .build();

        PathChain toScore3 = follower.pathBuilder()
                .addPath(new BezierLine(intake1donePose, shootPoselast))
                .setLinearHeadingInterpolation(intake1donePose.getHeading(), shootPoselast.getHeading())
                .build();
        PathChain toScore4 = follower.pathBuilder()
                .addPath(new BezierLine(intake4donePose, shootPoseleave))
                .setLinearHeadingInterpolation(intake4donePose.getHeading(), shootPoseleave.getHeading())
                .build();

        PathChain park = follower.pathBuilder()
                .addPath(new BezierLine(shootPoselast, leave))
                .setLinearHeadingInterpolation(shootPoselast.getHeading(), leave.getHeading())
                .build();


        follower.setStartingPose(startPose);


        StateMachine stateMachine = new StateMachineBuilder()
                .state(States.Intake)
                .onEnter(()->{
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
                .transitionTimed(0.5, States.Intake)
                .build();


        StateMachine autoMachine = new StateMachineBuilder() //Autonomia
                .state(AutoStates.PUSH)
                .onEnter(()->{
                    follower.followPath(toPush, true);
                    shooter.setHood(0.52);
                    shooter.setTargetVelocity(1360);
                    shooter.setTurretPos(shooter.convertDegreestoServoPos(137*Posmultiplier));
                    rapidFire=true;
                })

                .transition(()->follower.atParametricEnd())
                .transitionTimed(0.4)
                .state(AutoStates.MOVETOSHOOT1)
                .onEnter(()->{
                    follower.followPath(toShoot, true);
                    shooter.setHood(0.52);
                    shooter.setTargetVelocity(1400);
                    shooter.setTurretPos(shooter.convertDegreestoServoPos(137*Posmultiplier));
                })

                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.7)
                .state(AutoStates.wait1)
                .onEnter(()->{

                })
                .transitionTimed(1)
                .state(AutoStates.SHOOT1)
                .onEnter(()->{
                    shooterButton=true;
                })
                .transitionTimed(1)

                .state(AutoStates.MOVETOINTAKE1)
                .onEnter(()->{
                    follower.followPath(toIntake1, true);
                })

                .transition(()->follower.atParametricEnd())
                .transitionTimed(2)

                .state(AutoStates.INTAKE1)
                .onEnter(()->{
                    follower.followPath(toIntake1fin, true);
                })

                .transitionTimed(1)
                .state(AutoStates.MOVETOSHOOT2)
                .onEnter(()->{
                    shooter.setTurretPos(shooter.convertDegreestoServoPos(136*Posmultiplier));
                    follower.followPath(toScore1, true);
                })
                .transition(()->!follower.isBusy())
                .transitionTimed(2.5)

                .state(AutoStates.wait2)
                .onEnter(()->{
                })
                .transitionTimed(1)
                .state(AutoStates.SHOOT2)
                .onEnter(()->{
                    shooterButton=true;
                })

                .transitionTimed(1.2)
                .state(AutoStates.MOVETOINTAKE2)
                .onEnter(()->{
                    rapidFire=false;
                    follower.followPath(toIntake2, true);
                })

                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.3)

                .state(AutoStates.INTAKE2)
                .onEnter(()->{
                    follower.followPath(toIntake2fin, true);
                })

                .transitionTimed(1)

                .state(AutoStates.INTAKE2BACK)
                .onEnter(()->{
                    follower.followPath(intake2ToGate, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1)
                .state(AutoStates.GATE)
                .onEnter(()->{
                    shooter.setTurretPos(shooter.convertDegreestoServoPos(-45*Posmultiplier));
                    follower.followPath(opengatein, true);
                })
                .transitionTimed(2.3)
                .state(AutoStates.MOVETOSHOOT3)
                .onEnter(()->{
                    follower.followPath(toScore2, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.6)

                .state(AutoStates.wait3)
                .onEnter(()->{
                    if (pattern==1){
                        shootorder = new int[]{1, 2, 0};
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot2);
                    }else if (pattern==2){
                        shootorder = new int[]{0, 1, 2};
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot1);
                    }else{
                        shootorder = new int[]{2, 0, 1};
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot3);
                    }
                })
                .transitionTimed(0.3)
                .state(AutoStates.SHOOT3)
                .onEnter(()->{
                    shooterButton=true;
                })
                /*
                .transitionTimed(2)
                .transition(()->stateMachine.getStateEnum() == States.Intake)

                .state(AutoStates.MOVETOINTAKE3)
                .onEnter(()->{
                    follower.followPath(toIntake3, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(2)
                .state(AutoStates.INTAKE3)
                .onEnter(()->{
                    shooter.aimAtTarget(shootPoselast,shooterTarget);
                    follower.followPath(toIntake3fin, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(2.3)
                .state(AutoStates.MOVETOSHOOT4)
                .onEnter(()->{
                    follower.followPath(toScore3, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.7)
                .state(AutoStates.wait4)
                .onEnter(()->{
                    if (pattern==1){
                        shootorder = new int[]{0, 1, 2};
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot1);
                    }else if (pattern==2){
                        shootorder = new int[]{2, 0, 1};
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot3);
                    }else{
                        shootorder = new int[]{1, 2, 0};
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot2);
                    }
                    shooter.setHood(0.84);
                    shooter.setTurretPos(shooter.convertDegreestoServoPos(-108 * Posmultiplier));
                    shooter.setTargetVelocity(1490);
                })
                .transitionTimed(0.3)
                .state(AutoStates.SHOOT4)
                .onEnter(()->{
                    shooterButtonAll=true;
                })
                .transitionTimed(1.55)
                .transition(()->stateMachine.getStateEnum() == States.Intake)

                .state(AutoStates.MOVETOINTAKE4)
                .onEnter(()->{
                    follower.followPath(toIntake4, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(0.2)
                .state(AutoStates.INTAKE4)
                .onEnter(()->{
                    follower.followPath(toIntake4fin, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.7)
                .state(AutoStates.MOVETOSHOOT5)
                .onEnter(()->{
                    follower.followPath(toScore4, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(2)
                .state(AutoStates.wait5)
                .onEnter(()->{
                    if (pattern==1){
                        shootorder = new int[]{0, 1, 2};
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot1);
                    }else if (pattern==2){
                        shootorder = new int[]{2, 0, 1};
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot3);
                    }else{
                        shootorder = new int[]{1, 2, 0};
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot2);

                    }
                    shooter.setHood(0.84);
                    shooter.setTurretPos(shooter.convertDegreestoServoPos(-108 * Posmultiplier));
                    shooter.setTargetVelocity(1490);
                })
                .transitionTimed(0.3)
                .state(AutoStates.SHOOT5)
                .onEnter(()->{
                    shooterButtonAll=true;
                })
                .transitionTimed(2)
                .state(AutoStates.LEAVE)
                .onEnter(()->{
                    follower.followPath(park, true);
                }
                */
                .build();

        stateMachine.start();
        autoMachine.start();

        while (opModeIsActive()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();
            Position.pose = follower.getPose();
            stateMachine.update();
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

