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
@Autonomous(name = "AutoStartFarIndex18", group = "Auto")
public class AutoStartFarShootColsePush18 extends LinearOpMode {
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
    public double shootWaitTime = 0.28;
    public double intakeWaitTime = 0.35;

    public static boolean rapidFire = true;


    public static Shooter.Goal shooterTarget = Shooter.Goal.BLUE;


    public enum AutoStates {
        PUSH,
        MOVETOSHOOT1, wait1, SHOOT1,
        INTAKE1,
        MOVETOSHOOT2, wait2, SHOOT2,
        INTAKE2,
        MOVETOSHOOT3, wait3,SHOOT3,
        INTAKE3, BACK, GATE, waitgate,
        MOVETOSHOOT4, wait4,SHOOT4,
        INTAKE4,
        MOVETOSHOOT5, wait5,SHOOT5,
        INTAKE5,
        MOVETOSHOOT6, wait6,SHOOT6,
    }

    public enum States{
        BeforeIntake,
        Intake,


        Increment1,
        Wait1,
        Increment2,
        Wait2,

        BeforeWaitForShoot,
        WaitForShoot,

        Kick1,//spindex shoot
        ShootSpin1,
        Kick2,
        ShootSpin2,
        Kick3,

        OpenUpperGate,//rapid 3
        Shoot
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
            spindexer.setPosition(Spindexer.SpindexerPosition.Shoot2);
            shooter.setUpperGateOpen(false);
            spindexer.setLowerGateOpen(true);
            spindexer.setKickerPos(false);
            shooter.setTurretPos(shooter.convertDegreestoServoPos(0));
            shooter.setHood(0.7);
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
            shooter.update();
        }

        waitForStart();


        Pose opengate = new Pose(2, -57*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose opengateback = new Pose(3, -42*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose startPose = new Pose(60, -15*Posmultiplier, Math.toRadians(180*Posmultiplier));
        Pose pushPose = new Pose(58, -24*Posmultiplier, Math.toRadians(180*Posmultiplier));
        Pose shootPosepreload = new Pose(45, -12*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose shootPose1 = new Pose(-8, -14*Posmultiplier, Math.toRadians(-20*Posmultiplier));
        Pose shootPose2 = new Pose(-9, -16*Posmultiplier, Math.toRadians(-30*Posmultiplier));
        Pose shootPose3 = new Pose(-5, -19*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose shootPose4 = new Pose(-22, -22*Posmultiplier, Math.toRadians(-32*Posmultiplier));
        Pose shootPoseleave = new Pose(-39, -16*Posmultiplier, Math.toRadians(-29*Posmultiplier));

        Pose intake1backControl = new Pose(40, -20*Posmultiplier, Math.toRadians(-90*Posmultiplier));

        Pose intake1Pose = new Pose(34, -34*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose intake2ControlPose = new Pose(44, -61*Posmultiplier, Math.toRadians(-24*Posmultiplier));
        Pose intake2Pose = new Pose(30, -30*Posmultiplier, Math.toRadians(-24*Posmultiplier));
        Pose intake3Pose = new Pose(10,-34*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose intake4Pose = new Pose(-11,-34*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose intake5Pose = new Pose(26,-56*Posmultiplier, Math.toRadians(-20*Posmultiplier));

        Pose intake1donePose = new Pose(34, -64*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose intake2donePose = new Pose(61, -63*Posmultiplier, Math.toRadians(0*Posmultiplier));
        Pose intake3donePose = new Pose(11, -64*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose intake4donePose = new Pose(-14, -56*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose intake5donePose = new Pose(56,-62*Posmultiplier, Math.toRadians(-10*Posmultiplier));


        PathChain toPush = follower.pathBuilder()
                .addPath(new BezierLine(startPose, pushPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), pushPose.getHeading())
                .setBrakingStrength(1.5)
                .build();
        PathChain toShootpreload = follower.pathBuilder()
                .addPath(new BezierLine(pushPose, shootPosepreload))
                .setLinearHeadingInterpolation(pushPose.getHeading(), shootPosepreload.getHeading())
                .setBrakingStrength(1.5)
                .build();
        PathChain toIntake1 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPosepreload, intake1Pose,intake1donePose))
                .setLinearHeadingInterpolation(shootPosepreload.getHeading(), intake1donePose.getHeading())
                .build();
        PathChain toShoot1 = follower.pathBuilder()
                .addPath(new BezierCurve(intake1donePose, intake1backControl,shootPose1))
                .setTangentHeadingInterpolation()
                .setReversed()
                .setBrakingStrength(1.5)
                .build();
        PathChain toIntake2 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose1, intake2Pose,intake2ControlPose, intake2donePose))
                .setTangentHeadingInterpolation()
                .build();
        PathChain toShoot2 = follower.pathBuilder()
                .addPath(new BezierCurve(intake2donePose, intake2ControlPose,intake2Pose,shootPose2))
                .setTangentHeadingInterpolation()
                .setReversed()
                .setBrakingStrength(1.5)
                .build();
        PathChain toIntake3 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose2, intake3Pose,intake3donePose))
                .setLinearHeadingInterpolation(shootPose2.getHeading(), intake3donePose.getHeading())
                .build();
        PathChain toGateBack = follower.pathBuilder()
                .addPath(new BezierLine(intake3donePose, opengateback))
                .setLinearHeadingInterpolation(intake3donePose.getHeading(), opengateback.getHeading())
                .build();
        PathChain toGateForward = follower.pathBuilder()
                .addPath(new BezierLine(opengateback, opengate))
                .setLinearHeadingInterpolation(opengateback.getHeading(), opengate.getHeading())
                .build();
        PathChain toShoot3 = follower.pathBuilder()
                .addPath(new BezierLine(opengate, shootPose3))
                .setLinearHeadingInterpolation(opengate.getHeading(), shootPose3.getHeading())
                .setBrakingStrength(1.5)
                .build();
        PathChain toIntake4 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose3, intake4Pose,intake4donePose))
                .setLinearHeadingInterpolation(shootPose3.getHeading(), intake4donePose.getHeading())
                .build();
        PathChain toShoot4 = follower.pathBuilder()
                .addPath(new BezierLine(intake4donePose, shootPose4))
                .setLinearHeadingInterpolation(intake4donePose.getHeading(), shootPose4.getHeading())
                .setBrakingStrength(1.5)
                .build();
        PathChain toIntake5 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose4, intake5Pose,intake5donePose))
                .setLinearHeadingInterpolation(shootPose4.getHeading(), intake5donePose.getHeading())
                .build();
        PathChain toShootleave = follower.pathBuilder()
                .addPath(new BezierLine(intake4donePose, shootPoseleave))
                .setTangentHeadingInterpolation()
                .setReversed()
                .setBrakingStrength(1.5)
                .build();

        follower.setStartingPose(startPose);


        StateMachine stateMachine = new StateMachineBuilder()
                .state(States.BeforeIntake)
                .loop(()->{
                    if (rapidFire) {
                        if (intakes.getGoodBeamBreakOutside() && intakes.getGoodBeamBreakInside() && intakes.getGoodIntakeDetected()) {
                            System.out.println("all detected");
                            intakes.setGoodIntakePower(0.2);
                        } else {
                            intakes.setGoodIntakePower(1);
                        }
                    }else{
                        intakes.setGoodIntakePower(1);
                    }
                    intakes.setBadIntakePower(-0.2);
                    shooter.setUpperGateOpen(false);
                    spindexer.setLowerGateOpen(rapidFire);
                    spindexer.setKickerPos(false);
                    spindexer.setPosition(Spindexer.SpindexerPosition.Shoot2);
                })
                .transition(()->shooterButton, States.WaitForShoot)
                .transitionTimed(0.4, States.Intake)


                .state(States.Intake)
                .loop(()->{
                    if (rapidFire) {
                        if (intakes.getGoodBeamBreakOutside() && intakes.getGoodBeamBreakInside() && intakes.getGoodIntakeDetected()) {
                            System.out.println("all detected");
                            intakes.setGoodIntakePower(0.2);
                        } else {
                            intakes.setGoodIntakePower(1);
                        }
                    }else{
                        intakes.setGoodIntakePower(1);
                    }
                    intakes.setBadIntakePower(-0.2);
                    shooter.setUpperGateOpen(false);
                    spindexer.setLowerGateOpen(rapidFire);
                    spindexer.setKickerPos(false);
                    spindexer.setPosition(Spindexer.SpindexerPosition.Shoot2);
                })
                .transition(()->shooterButton, States.WaitForShoot)
                .transition(()->!rapidFire && intakes.getGoodIntakeDetected(), States.Wait1)

                .state(States.Wait1)
                .onEnter(()->{
                    spindexer.setPosition(Spindexer.SpindexerPosition.Shoot1);
                })
                .transitionTimed(intakeWaitTime, States.Increment1)
                .transition(()->shooterButton, States.WaitForShoot)

                .state(States.Increment1)
                .onEnter(()->{
                    spindexer.setPosition(Spindexer.SpindexerPosition.Shoot1);
                })
                .transition(()->intakes.getGoodIntakeDetected(), States.Wait2)
                .transition(()->shooterButton, States.WaitForShoot)

                .state(States.Wait2)
                .onEnter(()->{
                    spindexer.setPosition(Spindexer.SpindexerPosition.Shoot0);
                    intakes.setBadIntakePower(0.3);
                })
                .transitionTimed(intakeWaitTime, States.Increment2)
                .transition(()->shooterButton, States.WaitForShoot)

                .state(States.Increment2)
                .onEnter(()->{
                    spindexer.setPosition(Spindexer.SpindexerPosition.Shoot0);
                })
                .transition(()->intakes.getGoodIntakeDetected(), States.WaitForShoot)
                .transition(()->shooterButton, States.WaitForShoot)

                .state(States.WaitForShoot)
                .onEnter(()->{
                    if (shootorder[0] == 2) {
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot0);
                    }else if (shootorder[0] == 1) {
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot1);
                    }else if (shootorder[0] == 0) {
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot2);
                    }
                })
                .transition(()->shooterButton && rapidFire, States.OpenUpperGate)
                .transition(()->shooterButton && !rapidFire, States.Kick1)
                .onExit(()->{
                    shooterButton = false;
                })


                .state(States.OpenUpperGate)
                .onEnter(()->{
                    intakes.setGoodIntakePower(1);
                    spindexer.setLowerGateOpen(true);
                    shooter.setUpperGateOpen(true);
                })
                .transitionTimed(0.2)

                .state(States.Shoot)
                .onEnter(()->{
                    spindexer.setKickerPos(true);
                })
                .transitionTimed(0.4, States.Intake)

                .state(States.Kick1)
                .onEnter(()->{
                    if (shootorder[0] == 2) {
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot0);
                    }else if (shootorder[0] == 1) {
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot1);
                    }else if (shootorder[0] == 0) {
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot2);
                    }
                    spindexer.setLowerGateOpen(true);
                    shooter.setUpperGateOpen(true);
                    spindexer.setKickerPos(true);
                    intakes.setGoodIntakePower(1);
                })
                .transitionTimed(shootWaitTime, States.ShootSpin1)

                .state(States.ShootSpin1)
                .onEnter(()->{
                    if (shootorder[1] == 2) {
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot0);
                    }else if (shootorder[1] == 1) {
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot1);
                    }else if (shootorder[1] == 0) {
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot2);
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
                    if (shootorder[2] == 2) {
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot0);
                    }else if (shootorder[2] == 1) {
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot1);
                    }else if (shootorder[2] == 0) {
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot2);
                    }
                    spindexer.setKickerPos(false);
                })
                .transitionTimed(shootWaitTime, States.Kick3)

                .state(States.Kick3)
                .onEnter(()->{
                    spindexer.setKickerPos(true);
                })
                .transitionTimed(shootWaitTime, States.BeforeIntake)
                .build();


        StateMachine autoMachine = new StateMachineBuilder() //Autonomia
                .state(AutoStates.PUSH)
                .onEnter(()->{
                    follower.followPath(toPush, true);
                    shooter.setHood(0.8);
                    shooter.setTargetVelocity(2000);
                    shooter.setTurretPos(shooter.convertDegreestoServoPos(50*Posmultiplier));
                    rapidFire=true;
                })

                .transition(()->follower.atParametricEnd())
                .transitionTimed(0.3)
                .state(AutoStates.MOVETOSHOOT1)
                .onEnter(()->{
                    follower.followPath(toShootpreload, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1)

                .state(AutoStates.wait1)
                .transitionTimed(0.3)

                .state(AutoStates.SHOOT1)
                .onEnter(()->{
                    shooterButton=true;
                })
                .transitionTimed(0.6)

                .state(AutoStates.INTAKE1)
                .onEnter(()->{
                    follower.followPath(toIntake1, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.5)

                .state(AutoStates.MOVETOSHOOT2)
                .onEnter(()->{
                    shooter.setHood(0.67);
                    shooter.setTargetVelocity(1300);
                    shooter.setTurretPos(shooter.convertDegreestoServoPos(90*Posmultiplier));
                    follower.followPath(toShoot1, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(2.5)
                .state(AutoStates.wait2)
                .transitionTimed(0.2)
                .state(AutoStates.SHOOT2)
                .onEnter(()->{
                    shooterButton=true;
                })
                .transitionTimed(1.2)
                .state(AutoStates.INTAKE2)
                .onEnter(()->{
                    follower.followPath(toIntake2, true);
                })

                .transition(()->follower.atParametricEnd())
                .transitionTimed(2)
                .state(AutoStates.MOVETOSHOOT3)
                .onEnter(()->{
                    shooter.setHood(0.67);
                    shooter.setTargetVelocity(1300);
                    shooter.setTurretPos(shooter.convertDegreestoServoPos(140*Posmultiplier));
                    follower.followPath(toShoot2, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(2.5)
                .state(AutoStates.wait3)
                .transitionTimed(0.2)
                .state(AutoStates.SHOOT3)
                .onEnter(()->{
                    rapidFire=false;
                    shooterButton=true;
                })
                .transitionTimed(1.2)
                .state(AutoStates.INTAKE3)
                .onEnter(()->{
                    follower.followPath(toIntake3, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(2)
                .state(AutoStates.BACK)
                .onEnter(()->{
                    follower.followPath(toGateBack, true);
                })
                .transitionTimed(0.3)
                .state(AutoStates.GATE)
                .onEnter(()->{
                    follower.followPath(toGateForward, true);
                })
                .transitionTimed(0.4)
                .state(AutoStates.waitgate)
                .onEnter(()->{
                })
                .transitionTimed(1.5)

                .state(AutoStates.MOVETOSHOOT4)
                .onEnter(()->{
                    follower.followPath(toShoot3, true);
                    if (pattern==1){
                        shootorder = new int[]{1, 2, 0};
                    }else if (pattern==2){
                        shootorder = new int[]{0, 1, 2};
                    }else{
                        shootorder = new int[]{2, 0, 1};
                    }
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.2)

                .state(AutoStates.wait4)
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
                .transitionTimed(0.4)
                .state(AutoStates.SHOOT4)
                .onEnter(()->{
                    shooterButton=true;
                })
                .transitionTimed(1.67)

                .state(AutoStates.INTAKE4)
                .onEnter(()->{
                    follower.followPath(toIntake4, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(3)

                .state(AutoStates.MOVETOSHOOT5)
                .onEnter(()->{
                    follower.followPath(toShoot4);

                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.7)

                .state(AutoStates.wait5)
                .onEnter(()->{
                    if (pattern==3){
                        shootorder = new int[]{0, 1, 2};
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot1);
                    }else if (pattern==1){
                        shootorder = new int[]{2, 0, 1};
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot3);
                    }else{
                        shootorder = new int[]{1, 2, 0};
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot2);
                    }
                })
                .transitionTimed(0.3)

                .state(AutoStates.SHOOT5)
                .onEnter(()->{
                    rapidFire=true;
                    shooterButton=true;
                })
                .transitionTimed(1.55)
                .transition(()->stateMachine.getStateEnum() == States.Intake)

                .state(AutoStates.INTAKE5)
                .onEnter(()->{
                    follower.followPath(toIntake5, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(2.3)

                .state(AutoStates.MOVETOSHOOT6)
                .onEnter(()->{
                    follower.followPath(toShootleave, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(2)
                .state(AutoStates.wait6)
                .onEnter(()->{
                })
                .transitionTimed(0.4)

                .state(AutoStates.SHOOT6)
                .onEnter(()->{
                    spindexer.setKickerPos(true);
                    shooterButton=true;
                })
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

