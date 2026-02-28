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
@Autonomous(name = "AutoStartFarIndexFULL12", group = "Auto")
public class AutoStartFarShootClosePush12 extends LinearOpMode {
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
    public double shootWaitTime = 0.5;
    public double intakeWaitTime = 0.35;

    public static boolean rapidFire = true;


    public enum AutoStates {
        PUSH,
        MOVETOSHOOT1, wait1, SHOOT1,
        INTAKE1, BACK, GATE, waitgate,
        MOVETOSHOOT2, wait2, SHOOT2,
        INTAKE2, INTAKE2INSLOW,
        MOVETOSHOOT3, wait3,SHOOT3,
        INTAKE3, INTAKE3INSLOW,
        MOVETOSHOOT4, wait4,SHOOT4, leave
    }

    public enum States{
        BeforeIntake,
        Intake,


        Increment1,
        Wait1,
        Increment2,
        Wait2,

        ToIntake2,
        PulseEject,
        BackToShoot0,

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

        rapidFire = true;

        while (opModeInInit()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();
            follower.update();
            int currpattern = limelightCamera.getMotif();
            spindexer.setPosition(Spindexer.SpindexerPosition.Shoot2);
            shooter.setUpperGateOpen(false);
            spindexer.setLowerGateOpen(true);
            spindexer.setKickerPos(false);
            shooter.setTurretPos(shooter.convertDegreestoServoPos(shooter.convertDegreestoServoPos(0*Posmultiplier)));
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
                Posmultiplier=1;
            }
            if (gamepad1.b){
                colorAlliance="RED";
                Posmultiplier=-1;
            }
            telemetry.update();
            spindexer.update();
            shooter.update();
        }

        waitForStart();


        Pose opengate = new Pose(-4, -58.5*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose opengateback = new Pose(-3, -40*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose startPose = new Pose(60, -15*Posmultiplier, Math.toRadians(180*Posmultiplier));
        Pose pushPose = new Pose(58, -24*Posmultiplier, Math.toRadians(180*Posmultiplier));
        Pose shootPose = new Pose(-8, -19*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose shootleave = new Pose(-40, -19*Posmultiplier, Math.toRadians(-90*Posmultiplier));

        Pose toShoot2Control = new Pose(10, -30*Posmultiplier, Math.toRadians(-90*Posmultiplier));

        Pose intake1Pose = new Pose(-14, -30*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose intake2Pose = new Pose(9, -30*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose intake3Pose = new Pose(32,-30*Posmultiplier, Math.toRadians(-90*Posmultiplier));

        Pose intake1donePose = new Pose(-14, -62*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose intake2donePose = new Pose(12, -63*Posmultiplier, Math.toRadians(-90*Posmultiplier));
        Pose intake3donePose = new Pose(32, -64*Posmultiplier, Math.toRadians(-90*Posmultiplier));


        PathChain toPush = follower.pathBuilder()
                .addPath(new BezierLine(startPose, pushPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), pushPose.getHeading())
                .setBrakingStrength(1.5)
                .build();
        PathChain toShootpreload = follower.pathBuilder()
                .addPath(new BezierCurve(pushPose, startPose,shootPose))
                .setLinearHeadingInterpolation(pushPose.getHeading(), shootPose.getHeading())
                .setBrakingStrength(1.5)
                .build();
        PathChain toIntake1 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, intake1Pose,intake1donePose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intake1donePose.getHeading())
                .build();
        PathChain toIntake1backgate = follower.pathBuilder()
                .addPath(new BezierLine(intake1donePose,opengateback))
                .setLinearHeadingInterpolation(intake1donePose.getHeading(), intake1Pose.getHeading())
                .build();
        PathChain ingate = follower.pathBuilder()
                .addPath(new BezierLine(opengateback,opengate))
                .setLinearHeadingInterpolation(opengateback.getHeading(), opengate.getHeading())
                .setBrakingStrength(0.4)
                .build();
        PathChain toShoot1 = follower.pathBuilder()
                .addPath(new BezierCurve(opengate,opengateback,shootPose))
                .setLinearHeadingInterpolation(opengate.getHeading(), shootPose.getHeading())
                .setBrakingStrength(1.5)
                .build();
        PathChain toIntake2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, intake2Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(),intake2Pose.getHeading())
                .build();
        PathChain toIntake2in = follower.pathBuilder()
                .addPath(new BezierLine(intake2Pose, intake2donePose))
                .setLinearHeadingInterpolation(intake2Pose.getHeading(),intake2donePose.getHeading())
                .build();

        PathChain toShoot2 = follower.pathBuilder()
                .addPath(new BezierCurve(intake2donePose,intake2Pose, shootPose))
                .setLinearHeadingInterpolation(intake2donePose.getHeading(),shootPose.getHeading())
                .setBrakingStrength(1.5)
                .build();
        PathChain toIntake3 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, intake3Pose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intake3Pose.getHeading())
                .build();
        PathChain toIntake3in = follower.pathBuilder()
                .addPath(new BezierLine(intake3Pose,intake3donePose))
                .setLinearHeadingInterpolation(intake3Pose.getHeading(), intake3donePose.getHeading())
                .build();
        PathChain toShoot3 = follower.pathBuilder()
                .addPath(new BezierCurve(intake3donePose, intake3Pose, shootleave))
                .setLinearHeadingInterpolation(opengate.getHeading(), shootleave.getHeading())
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
                .transition(()->intakes.getGoodIntakeDetected(), States.ToIntake2)
                .transition(()->shooterButton, States.WaitForShoot)

                .state(States.ToIntake2)
                .onEnter(()->{
                    spindexer.setPosition(Spindexer.SpindexerPosition.Intake2);
                })
                .transitionTimed(0.3, States.PulseEject)

                .state(States.PulseEject)
                .onEnter(()->{
                    intakes.setGoodIntakePower(-1);
                })
                .transitionTimed(0.4)

                .state(States.BackToShoot0)
                .onEnter(()->{
                    spindexer.setPosition(Spindexer.SpindexerPosition.Shoot0);
                    intakes.setGoodIntakePower(0.5);
                })
                .transitionTimed(0.3, States.WaitForShoot)

                .state(States.WaitForShoot)
                .onEnter(()->{
                    if (!rapidFire) {
                        if (shootorder[0] == 2) {
                            spindexer.setPosition(Spindexer.SpindexerPosition.Shoot0);
                        } else if (shootorder[0] == 1) {
                            spindexer.setPosition(Spindexer.SpindexerPosition.Shoot1);
                        } else if (shootorder[0] == 0) {
                            spindexer.setPosition(Spindexer.SpindexerPosition.Shoot2);
                        }
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
                    rapidFire=true;
                    follower.followPath(toPush, true);
                    shooter.setHood(0.63);
                    shooter.setTargetVelocity(1420);
                    shooter.setTurretPos(shooter.convertDegreestoServoPos(52*Posmultiplier));
                })

                .transition(()->follower.atParametricEnd())
                .transitionTimed(0.3)
                .state(AutoStates.MOVETOSHOOT1)
                .onEnter(()->{
                    follower.followPath(toShootpreload, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(2)

                .state(AutoStates.wait1)
                .onEnter(()->{
                })
                .transitionTimed(0.8)

                .state(AutoStates.SHOOT1)
                .onEnter(()->{
                    shooterButton=true;
                })
                .transitionTimed(1.2)

                .state(AutoStates.INTAKE1)
                .onEnter(()->{
                    rapidFire=false;
                    if (pattern==1){
                        shootorder = new int[]{2, 0, 1};
                    }else if (pattern==2){
                        shootorder = new int[]{1, 2, 0};
                    }else{
                        shootorder = new int[]{0, 1, 2};
                    }
                    follower.followPath(toIntake1, 0.5,true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.6)
                .state(AutoStates.BACK)
                .onEnter(()->{
                    follower.followPath(toIntake1backgate, true);
                })
                .transitionTimed(0.4)
                .state(AutoStates.GATE)
                .onEnter(()->{
                    shooter.setHood(0.63);
                    shooter.setTargetVelocity(1430);
                    shooter.setTurretPos(shooter.convertDegreestoServoPos(49*Posmultiplier));
                    follower.followPath(ingate, true);
                })
                .transitionTimed(0.7)
                .state(AutoStates.waitgate)
                .onEnter(()->{
                    intakes.setGoodIntakePower(0);
                })
                .transitionTimed(2)

                .state(AutoStates.MOVETOSHOOT2)
                .onEnter(()->{
                    intakes.setGoodIntakePower(1);
                    follower.followPath(toShoot1, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(2)
                .state(AutoStates.wait2)
                .transitionTimed(0.2)
                .state(AutoStates.SHOOT2)
                .onEnter(()->{
                    shooterButton=true;
                })
                .transitionTimed(3)
                .transition(()->stateMachine.getStateEnum() == States.Intake)
                .state(AutoStates.INTAKE2)
                .onEnter(()->{
                    follower.followPath(toIntake2, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.5)
                .state(AutoStates.INTAKE2INSLOW)
                .onEnter(()->{
                    if (pattern==1){
                        shootorder = new int[]{1, 2, 0};
                    }else if (pattern==2){
                        shootorder = new int[]{0, 1, 2};
                    }else{
                        shootorder = new int[]{2, 0, 1};
                    }
                    follower.followPath(toIntake2in, 0.5,true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(2.5)
                .state(AutoStates.MOVETOSHOOT3)
                .onEnter(()->{
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
                .transition(()->stateMachine.getStateEnum() == States.Intake)
                .transitionTimed(3)
                .state(AutoStates.INTAKE3)
                .onEnter(()->{
                    follower.followPath(toIntake3, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(2)
                .state(AutoStates.INTAKE3INSLOW)
                .onEnter(()->{
                    if (pattern==1){
                        shootorder = new int[]{0, 1, 2};
                    }else if (pattern==2){
                        shootorder = new int[]{2, 0, 1};
                    }else{
                        shootorder = new int[]{1, 2, 0};
                    }
                    follower.followPath(toIntake3in, 0.5,true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(2)
                .state(AutoStates.MOVETOSHOOT4)
                .onEnter(()->{
                    shooter.setHood(0.53);
                    shooter.setTargetVelocity(1250);
                    shooter.setTurretPos(shooter.convertDegreestoServoPos(29*Posmultiplier));
                    follower.followPath(toShoot3, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(2.6)
                .state(AutoStates.wait4)
                .onEnter(()->{
                })
                .transitionTimed(0.4)
                .state(AutoStates.SHOOT4)
                .onEnter(()->{
                    shooterButton=true;
                })
                .build();

        stateMachine.start();
        autoMachine.start();
        rapidFire=true;
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
            telemetry.addData("Spindexer position", spindexer.getCurrentPosition());
            telemetry.addData("rapidfire", rapidFire);
            telemetry.update();
        }
    }
}

