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
//import org.firstinspires.ftc.teamcode.subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.subsystems.Position;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

import java.util.Arrays;
import java.util.List;

@Configurable
@Autonomous(name = "AutoFarFarLLGateIntakeRun", group = "Auto")
public class AutoStartFarShootFarLL extends LinearOpMode {
    private Follower follower;
    Intakes intakes;
    String colorAlliance = "BLUE";
    int Posmultiplier = 1;
    Shooter shooter;
    LimelightCamera limelight;
    Spindexer spindexer;
    public boolean shooterButton = false;
    public double shootWaitTime = 0.25;
    public double intakerejectspeed = -0.1;
    public double limelightAdjust=0.0;



    public static Shooter.Goal shooterTarget = Shooter.Goal.BLUE;


    public enum AutoStates {
        MOVETOSHOOT1, wait1, preSHOOT1,SHOOT1,
        MOVETOINTAKE1,
        MOVETOSHOOT2, wait2, preSHOOT2,SHOOT2,
        MOVETOINTAKE2,
        MOVETOSHOOT3, wait3,preSHOOT3, SHOOT3,
        MOVETOINTAKE3,
        MOVETOSHOOT4, reject1, wait4,preSHOOT4, SHOOT4,
        MOVETOINTAKE4,
        MOVETOSHOOT5, reject2, wait5,preSHOOT5, SHOOT5,
        MOVETOINTAKE5,
        MOVETOSHOOT6, reject3,wait6,preSHOOT6, SHOOT6,
        MOVETOINTAKE6,
        MOVETOSHOOT7, reject4, wait7,preSHOOT7, SHOOT7,
        MOVETOINTAKE7,
        MOVETOSHOOT8, reject5, wait8,preSHOOT8, SHOOT8,
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
        limelight = new LimelightCamera(hardwareMap);

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
            spindexer.setPosition(Spindexer.SpindexerPosition.Shoot0);
            spindexer.setKickerPos(false);
            shooter.setTurretPos(shooter.convertDegreestoServoPos(0));
            shooter.setHood(0.8);
            shooter.setUpperGateOpen(false);
            spindexer.setLowerGateOpen(true);
            shooter.update();
            telemetry.update();
            spindexer.update();
        }
        if (opModeIsActive()) {
            if (shooterTarget == Shooter.Goal.BLUE) {
                limelight.setCurrentPipeline(LimelightCamera.Pipelines.BLUETRACK);
            } else {
                limelight.setCurrentPipeline(LimelightCamera.Pipelines.REDTRACK);
            }

            waitForStart();


            Pose startPose = new Pose(57, -12 * Posmultiplier, Math.toRadians(-90 * Posmultiplier));
            Pose shootPose = new Pose(53, -22 * Posmultiplier, Math.toRadians(-90 * Posmultiplier));
            Pose intakeHuman = new Pose(58, -61 * Posmultiplier, Math.toRadians(-90 * Posmultiplier));
            Pose intake1Pose = new Pose(20, -28 * Posmultiplier, Math.toRadians(-90 * Posmultiplier));
            Pose intake1donePose = new Pose(33, -61 * Posmultiplier, Math.toRadians(-90 * Posmultiplier));


            Pose intakemidPose = new Pose(44, -17 * Posmultiplier, Math.toRadians(-90 * Posmultiplier));
            Pose intakemiddonePose = new Pose(44, -63 * Posmultiplier, Math.toRadians(-90 * Posmultiplier));
            Pose leave = new Pose(33, -30 * Posmultiplier, Math.toRadians(-90 * Posmultiplier));


            follower.setStartingPose(startPose);

            StateMachine autoMachine = new StateMachineBuilder() //Autonomia
                    .state(AutoStates.MOVETOSHOOT1)
                    .onEnter(() -> {
                        intakes.setGoodIntakePower(1);
                        spindexer.setKickerPos(false);
                        PathChain toScore = follower.pathBuilder()
                                .addPath(new BezierLine(follower.getPose(), shootPose))
                                .setLinearHeadingInterpolation(follower.getHeading(), shootPose.getHeading())
                                .setBrakingStrength(3)
                                .build();
                        follower.followPath(toScore, true);
                        //shooter.setHood(0.6);
                        //shooter.setTargetVelocity(1860);
                        //shooter.setTurretPos(shooter.convertDegreestoServoPos(69*Posmultiplier));
                    })
                    .transition(() -> follower.atParametricEnd())
                    .transitionTimed(0.5)

                    .state(AutoStates.wait1)
                    .onEnter(() -> {
                        limelightAdjust = limelight.getTrackingResults();
                    })
                    .transitionTimed(2.2)
                    .state(AutoStates.preSHOOT1)
                    .onEnter(() -> {
                        intakes.setGoodIntakePower(1);
                    })
                    .transitionTimed(0.1)
                    .state(AutoStates.SHOOT1)
                    .onEnter(() -> {
                        shooter.setUpperGateOpen(true);
                        spindexer.setKickerPos(true);
                    })
                    .transitionTimed(0.3)


                    .state(AutoStates.MOVETOINTAKE1)
                    .onEnter(() -> {
                        intakes.setGoodIntakePower(1);
                        spindexer.setKickerPos(false);
                        shooter.setUpperGateOpen(false);
                        //shooter.setTurretPos(shooter.convertDegreestoServoPos(71 *Posmultiplier));
                        //shooter.setHood(0.6);
                        //shooter.setTargetVelocity(1860);

                        PathChain toIntake = follower.pathBuilder()
                                .addPath(new BezierCurve(follower.getPose(), intake1Pose, intake1donePose))
                                .setLinearHeadingInterpolation(follower.getHeading(), intake1donePose.getHeading())
                                .setBrakingStrength(3)
                                .build();
                        follower.followPath(toIntake, true);
                    })
                    .transitionTimed(1.5)

                    .state(AutoStates.MOVETOSHOOT2)
                    .onEnter(() -> {
                        intakes.setGoodIntakePower(1);
                        PathChain toScore = follower.pathBuilder()
                                .addPath(new BezierLine(follower.getPose(), shootPose))
                                .setLinearHeadingInterpolation(follower.getHeading(), shootPose.getHeading())
                                .setBrakingStrength(3)
                                .build();
                        follower.followPath(toScore, true);
                    })
                    .transition(() -> follower.atParametricEnd())
                    .transitionTimed(2.4)

                    .state(AutoStates.wait2)
                    .onEnter(() -> {
                    })
                    .transitionTimed(0.5)

                    .state(AutoStates.preSHOOT2)
                    .onEnter(() -> {
                        intakes.setGoodIntakePower(1);
                    })
                    .transitionTimed(0.1)
                    .state(AutoStates.SHOOT2)
                    .onEnter(() -> {
                        shooter.setUpperGateOpen(true);
                        spindexer.setKickerPos(true);
                    })
                    .transitionTimed(0.55)
                    .state(AutoStates.MOVETOINTAKE2)
                    .onEnter(() -> {
                        intakes.setGoodIntakePower(1);
                        shooter.setUpperGateOpen(false);
                        spindexer.setKickerPos(false);

                        //shooter.setTurretPos(shooter.convertDegreestoServoPos(69*Posmultiplier));

                        PathChain toIntakeHuman = follower.pathBuilder()
                                .addPath(new BezierLine(follower.getPose(), intakeHuman))
                                .setLinearHeadingInterpolation(follower.getHeading(), intakeHuman.getHeading())
                                .build();
                        follower.followPath(toIntakeHuman, true);
                    })
                    .transitionTimed(1.2)

                    .state(AutoStates.MOVETOSHOOT3)
                    .onEnter(() -> {
                        intakes.setGoodIntakePower(1);
                        PathChain toScore = follower.pathBuilder()
                                .addPath(new BezierLine(follower.getPose(), shootPose))
                                .setLinearHeadingInterpolation(follower.getHeading(), shootPose.getHeading())
                                .setBrakingStrength(3)
                                .build();
                        follower.followPath(toScore, true);
                    })
                    .transition(() -> follower.atParametricEnd())
                    .transitionTimed(2)

                    .state(AutoStates.wait3)
                    .onEnter(() -> {
                    })
                    .transitionTimed(0.2)

                    .state(AutoStates.preSHOOT3)
                    .onEnter(() -> {
                        intakes.setGoodIntakePower(1);
                    })
                    .transitionTimed(0.1)
                    .state(AutoStates.SHOOT3)
                    .onEnter(() -> {
                        shooter.setUpperGateOpen(true);
                        spindexer.setKickerPos(true);
                    })
                    .transitionTimed(0.55)
                    .state(AutoStates.MOVETOINTAKE3)
                    .onEnter(() -> {
                        intakes.setGoodIntakePower(1);
                        //shooter.setTurretPos(shooter.convertDegreestoServoPos(69*Posmultiplier));
                        shooter.setUpperGateOpen(false);
                        spindexer.setKickerPos(false);
                    /*
                    // LLFieldScannerResults results = limelightCamera.getTrackingResults();
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
                    */

                        PathChain toIntake = follower.pathBuilder()
                                .addPath(new BezierLine(follower.getPose(), intakemiddonePose))
                                .setLinearHeadingInterpolation(follower.getHeading(), intakemiddonePose.getHeading())
                                .setNoDeceleration()
                                .build();
                        follower.followPath(toIntake, true);
                        System.out.println("Detected at NULL");

                    })
                    .transitionTimed(1.2)
                    .state(AutoStates.MOVETOSHOOT4)
                    .onEnter(() -> {
                        intakes.setGoodIntakePower(1);
                        PathChain toScore = follower.pathBuilder()
                                .addPath(new BezierLine(follower.getPose(), shootPose))
                                .setLinearHeadingInterpolation(follower.getHeading(), shootPose.getHeading())
                                .setBrakingStrength(3)
                                .build();
                        follower.followPath(toScore, true);
                    })
                    .transitionTimed(1)
                    .state(AutoStates.reject1)
                    .loop(() -> {
                        if (intakes.getGoodBeamBreakOutside() && intakes.getGoodBeamBreakInside() && intakes.getGoodIntakeDetected()){
                            intakes.setGoodIntakePower(intakerejectspeed);
                        }
                    })
                    .transition(() -> follower.atParametricEnd())
                    .transitionTimed(1.5)

                    .state(AutoStates.wait4)
                    .onEnter(() -> {
                    })
                    .transitionTimed(0.2)

                    .state(AutoStates.preSHOOT4)
                    .onEnter(() -> {
                        intakes.setGoodIntakePower(1);
                    })
                    .transitionTimed(0.1)
                    .state(AutoStates.SHOOT4)
                    .onEnter(() -> {
                        shooter.setUpperGateOpen(true);
                        spindexer.setKickerPos(true);
                    })
                    .transitionTimed(0.55)
                    .state(AutoStates.MOVETOINTAKE4)
                    .onEnter(() -> {
                        intakes.setGoodIntakePower(1);
                        //shooter.setTurretPos(shooter.convertDegreestoServoPos(69*Posmultiplier));
                        shooter.setUpperGateOpen(false);
                        spindexer.setKickerPos(false);
                    /*
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

                     */
                        PathChain toIntake = follower.pathBuilder()
                                .addPath(new BezierLine(follower.getPose(), intake1donePose))
                                .setTangentHeadingInterpolation()
                                .setNoDeceleration()
                                .build();
                        follower.followPath(toIntake, true);
                        System.out.println("Detected at NULL");

                    })
                    .transitionTimed(1.5)
                    .state(AutoStates.MOVETOSHOOT5)
                    .onEnter(() -> {
                        intakes.setGoodIntakePower(1);
                        PathChain toScore = follower.pathBuilder()
                                .addPath(new BezierLine(follower.getPose(), shootPose))
                                .setLinearHeadingInterpolation(follower.getHeading(), shootPose.getHeading())
                                .setBrakingStrength(0.7)
                                .build();
                        follower.followPath(toScore, true);
                    })
                    .transitionTimed(1)
                    .state(AutoStates.reject2)
                    .loop(() -> {
                        if (intakes.getGoodBeamBreakOutside() && intakes.getGoodBeamBreakInside() && intakes.getGoodIntakeDetected()){
                            intakes.setGoodIntakePower(intakerejectspeed);
                        }
                    })
                    .transition(() -> follower.atParametricEnd())
                    .transitionTimed(1.5)

                    .state(AutoStates.wait5)
                    .onEnter(() -> {
                    })
                    .transitionTimed(0.2)

                    .state(AutoStates.preSHOOT5)
                    .onEnter(() -> {
                        intakes.setGoodIntakePower(1);
                    })
                    .transitionTimed(0.1)
                    .state(AutoStates.SHOOT5)
                    .onEnter(() -> {
                        shooter.setUpperGateOpen(true);
                        spindexer.setKickerPos(true);
                    })
                    .transitionTimed(0.55)
                    .state(AutoStates.MOVETOINTAKE5)
                    .onEnter(() -> {
                        intakes.setGoodIntakePower(1);
                        //shooter.setTurretPos(shooter.convertDegreestoServoPos(69*Posmultiplier));
                        shooter.setUpperGateOpen(false);
                        spindexer.setKickerPos(false);
                    /*
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

                     */
                        PathChain toIntake = follower.pathBuilder()
                                .addPath(new BezierLine(follower.getPose(), intakeHuman))
                                .setTangentHeadingInterpolation()
                                .setNoDeceleration()
                                .build();
                        follower.followPath(toIntake, true);
                        System.out.println("Detected at NULL");

                    })
                    .transitionTimed(1.5)

                    .state(AutoStates.MOVETOSHOOT6)
                    .onEnter(() -> {
                        intakes.setGoodIntakePower(1);
                        PathChain toScore = follower.pathBuilder()
                                .addPath(new BezierLine(follower.getPose(), shootPose))
                                .setLinearHeadingInterpolation(follower.getHeading(), shootPose.getHeading())
                                .setBrakingStrength(0.7)
                                .build();
                        //shooter.setTurretPos(shooter.convertDegreestoServoPos(69*Posmultiplier));

                        follower.followPath(toScore, true);
                    })
                    .transitionTimed(1)
                    .state(AutoStates.reject3)
                    .loop(() -> {
                        if (intakes.getGoodBeamBreakOutside() && intakes.getGoodBeamBreakInside() && intakes.getGoodIntakeDetected()){
                            intakes.setGoodIntakePower(intakerejectspeed);
                        }
                    })
                    .transition(() -> follower.atParametricEnd())
                    .transitionTimed(1.5)

                    .state(AutoStates.wait6)
                    .onEnter(() -> {
                    })
                    .transitionTimed(0.2)

                    .state(AutoStates.preSHOOT6)
                    .onEnter(() -> {
                        intakes.setGoodIntakePower(1);
                    })
                    .transitionTimed(0.1)
                    .state(AutoStates.SHOOT6)
                    .onEnter(() -> {
                        shooter.setUpperGateOpen(true);
                        spindexer.setKickerPos(true);
                    })
                    .transitionTimed(0.55)
                    .state(AutoStates.MOVETOINTAKE6)
                    .onEnter(() -> {
                        intakes.setGoodIntakePower(1);
                        shooter.setUpperGateOpen(false);
                        spindexer.setKickerPos(false);
                    /*
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

                     */
                        PathChain toIntake = follower.pathBuilder()
                                .addPath(new BezierLine(follower.getPose(), intakemiddonePose))
                                .setTangentHeadingInterpolation()
                                .setNoDeceleration()

                                .build();
                        follower.followPath(toIntake, true);
                        System.out.println("Detected at NULL");

                    })
                    .transitionTimed(1.6)
                    .state(AutoStates.MOVETOSHOOT7)
                    .onEnter(() -> {
                        intakes.setGoodIntakePower(1);
                        PathChain toScore = follower.pathBuilder()
                                .addPath(new BezierLine(follower.getPose(), shootPose))
                                .setLinearHeadingInterpolation(follower.getHeading(), shootPose.getHeading())
                                .setBrakingStrength(0.7)
                                .build();
                        //shooter.setTurretPos(shooter.convertDegreestoServoPos(71*Posmultiplier));

                        follower.followPath(toScore, true);
                    })
                    .transitionTimed(1)
                    .state(AutoStates.reject4)
                    .loop(() -> {
                        if (intakes.getGoodBeamBreakOutside() && intakes.getGoodBeamBreakInside() && intakes.getGoodIntakeDetected()){
                            intakes.setGoodIntakePower(intakerejectspeed);
                        }
                    })
                    .transition(() -> follower.atParametricEnd())
                    .transitionTimed(1.5)

                    .state(AutoStates.wait7)
                    .onEnter(() -> {
                    })
                    .transitionTimed(0.2)

                    .state(AutoStates.preSHOOT7)
                    .onEnter(() -> {
                        intakes.setGoodIntakePower(1);
                    })
                    .transitionTimed(0.1)
                    .state(AutoStates.SHOOT7)
                    .onEnter(() -> {
                        shooter.setUpperGateOpen(true);
                        spindexer.setKickerPos(true);
                    })
                    .transitionTimed(0.55)
                    .state(AutoStates.MOVETOINTAKE7)
                    .onEnter(() -> {
                        intakes.setGoodIntakePower(1);
                        shooter.setUpperGateOpen(false);
                        spindexer.setKickerPos(false);
                    /*
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

                     */
                        PathChain toIntake = follower.pathBuilder()
                                .addPath(new BezierLine(follower.getPose(), intakeHuman))
                                .setTangentHeadingInterpolation()
                                .setNoDeceleration()

                                .build();
                        follower.followPath(toIntake, true);
                        System.out.println("Detected at NULL");

                    })
                    .transitionTimed(1.6)
                    .state(AutoStates.MOVETOSHOOT8)
                    .onEnter(() -> {
                        intakes.setGoodIntakePower(1);
                        PathChain toScore = follower.pathBuilder()
                                .addPath(new BezierLine(follower.getPose(), shootPose))
                                .setLinearHeadingInterpolation(follower.getHeading(), shootPose.getHeading())
                                .setBrakingStrength(0.7)
                                .build();
                        //shooter.setTurretPos(shooter.convertDegreestoServoPos(71*Posmultiplier));

                        follower.followPath(toScore, true);
                    })
                    .transitionTimed(1)
                    .state(AutoStates.reject5)
                    .loop(() -> {
                        if (intakes.getGoodBeamBreakOutside() && intakes.getGoodBeamBreakInside() && intakes.getGoodIntakeDetected()){
                            intakes.setGoodIntakePower(intakerejectspeed);
                        }
                    })
                    .transition(() -> follower.atParametricEnd())
                    .transitionTimed(1.5)

                    .state(AutoStates.wait8)
                    .onEnter(() -> {
                    })
                    .transitionTimed(0.2)

                    .state(AutoStates.preSHOOT8)
                    .onEnter(() -> {
                        intakes.setGoodIntakePower(1);
                    })
                    .transitionTimed(0.1)
                    .state(AutoStates.SHOOT8)
                    .onEnter(() -> {
                        shooter.setUpperGateOpen(true);
                        spindexer.setKickerPos(true);
                    })
                    .transitionTimed(0.55)
                    .state(AutoStates.park)
                    .onEnter(() -> {
                        shooter.setUpperGateOpen(false);
                        spindexer.setKickerPos(false);
                        PathChain park = follower.pathBuilder()
                                .addPath(new BezierLine(follower.getPose(), leave))
                                .setLinearHeadingInterpolation(follower.getHeading(), leave.getHeading())
                                .setBrakingStrength(0.6)
                                .build();
                        follower.followPath(park, true);
                    })
                    .build();

            autoMachine.start();
            //limelightCamera.setCurrentPipeline(LimelightCamera.Pipelines.BALLTRACKING);
            while (opModeIsActive()) {
                for (LynxModule hub : hubs) hub.clearBulkCache();
                Position.pose = follower.getPose();
                if (Posmultiplier == 1) {
                    Shooter.powerOffset = -50;
                    Shooter.turretOffset = 0;
                } else {
                    Shooter.powerOffset = -50;
                    Shooter.turretOffset = 0;
                }
                autoMachine.update();
                telemetry.addData("Angle and distance:", Arrays.toString(shooter.getAngleDistance(Position.pose, shooterTarget)));
                 shooter.setTurretPos(shooter.convertDegreestoServoPos(71*Posmultiplier+limelightAdjust - Math.toDegrees(follower.getHeadingError())));
                shooter.setTargetVelocity(1760);
                shooter.setHood(0.5);
                follower.update();
                intakes.update();
                shooter.update();
                spindexer.update();
                telemetry.addData("heading +90: ", follower.getHeading()*Posmultiplier);
                telemetry.addData("heading error: ", Math.toDegrees(follower.getHeadingError()));
                telemetry.addData("State auto: ", autoMachine.getState());
                telemetry.addData("Shooter Target", shooter.getTargetVelo());
                telemetry.addData("Shooter Velocity", shooter.getCurrentVelocity());
                telemetry.addData("Spindexer kick", spindexer.is_kick);
                telemetry.addData("Pose: ", follower.getPose());
                telemetry.update();
            }
        }
    }
}

