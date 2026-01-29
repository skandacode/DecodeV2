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
import org.firstinspires.ftc.teamcode.subsystems.LLFieldScannerResults;
import org.firstinspires.ftc.teamcode.subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.subsystems.Position;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

import java.util.List;

@Configurable
@Autonomous(name = "AutoFarFarLLLLLLLL", group = "Auto")
public class AutoStartFarLL extends LinearOpMode {
    private Follower follower;
    Intakes intakes;
    String colorAlliance = "BLUE";
    LimelightCamera limelightCamera;
    int Posmultiplier = 1;
    Shooter shooter;
    Spindexer spindexer;
    public boolean shooterButton = false;
    public double shootWaitTime = 0.43;


    public static Shooter.Goal shooterTarget = Shooter.Goal.BLUE;


    public enum AutoStates {
        MOVETOSHOOT1, wait1, SHOOT1,
        MOVETOINTAKE1, Out1, INTAKE1,
        MOVETOSHOOT2, wait2, SHOOT2,
        MOVETOINTAKE2, Out2, INTAKE2,
        MOVETOSHOOT3, wait3,SHOOT3,
        MOVETOINTAKE3, Out3, INTAKE3,
        MOVETOSHOOT4, wait4,SHOOT4,
        MOVETOINTAKE4, Out4,INTAKE4,
        MOVETOSHOOT5, wait5,SHOOT5,
        MOVETOINTAKE5, Out5,INTAKE5,
        MOVETOSHOOT6, wait6,SHOOT6,
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
        limelightCamera = new LimelightCamera(hardwareMap);

        follower = createFollower(hardwareMap);

        while (opModeInInit()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();
            follower.update();
            spindexer.setPosition(Spindexer.SpindexerPosition.Shoot2);
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


        Pose startPose = new Pose(60, -15*Posmultiplier, Math.toRadians(90*Posmultiplier));
        Pose shootPose = new Pose(53, -12*Posmultiplier, Math.toRadians(90*Posmultiplier));

        Pose intakeHuman = new Pose(61, -63*Posmultiplier, Math.toRadians(90*Posmultiplier));
        Pose intakeHumanOut = new Pose(59, -60*Posmultiplier, Math.toRadians(90*Posmultiplier));
        Pose intake1Pose = new Pose(31, -17*Posmultiplier, Math.toRadians(90*Posmultiplier));

        Pose intake1donePose = new Pose(33, -63*Posmultiplier, Math.toRadians(90*Posmultiplier));
        Pose leave = new Pose(40, -30*Posmultiplier, Math.toRadians(90*Posmultiplier));



        follower.setStartingPose(startPose);


        StateMachine stateMachine = new StateMachineBuilder()
                .state(States.Intake)
                .loop(()->{
                    shooter.setUpperGateOpen(true);
                    spindexer.setLowerGateOpen(false);
                    spindexer.setKickerPos(false);
                    spindexer.setPosition(Spindexer.SpindexerPosition.Intake1);
                    intakes.setGoodIntakePower(0.2);
                    intakes.setBadIntakePower(1);
                })
                .transition(()->shooterButton, States.WaitForShoot, ()->{
                    System.out.println("Transitioned from intake to wait for shoot because shooter button pressed and not rapid fire");
                })
                .transition(()->intakes.getBadIntakeDetected(), States.Wait1, ()->{
                    System.out.println("Transitioned from intake to wait1 because bad intake detected and not rapid fire");
                })

                .state(States.Wait1)
                .onEnter(()->{
                    spindexer.setPosition(Spindexer.SpindexerPosition.Intake2);
                })
                .transitionTimed(shootWaitTime, States.Increment2, ()->{
                    System.out.println("Transitioned from wait1 to increment2 because time elapsed");
                })
                .transition(()->shooterButton, States.WaitForShoot, ()->{
                    System.out.println("Transitioned from wait1 to wait for shoot because shooter button pressed");
                })

                .state(States.Increment2)
                .onEnter(()->{
                    spindexer.setPosition(Spindexer.SpindexerPosition.Intake2);
                })
                .transition(()->intakes.getBadIntakeDetected(), States.Wait2, ()->{
                    System.out.println("Transitioned from increment2 to wait2 because bad intake detected");
                })
                .transition(()->shooterButton, States.WaitForShoot, ()->{
                    System.out.println("Transitioned from increment2 to wait for shoot because shooter button pressed");
                })

                .state(States.Wait2)
                .onEnter(()->{
                    spindexer.setPosition(Spindexer.SpindexerPosition.Intake3);
                })
                .transitionTimed(shootWaitTime, States.Increment3, ()->{
                    System.out.println("Transitioned from wait2 to increment3 because time elapsed");
                })
                .transition(()->shooterButton, States.WaitForShoot, ()->{
                    System.out.println("Transitioned from wait2 to wait for shoot because shooter button pressed");
                })

                .state(States.Increment3)
                .onEnter(()->{
                    spindexer.setPosition(Spindexer.SpindexerPosition.Intake3);
                })
                .transition(()->intakes.getBadIntakeDetected(), States.WaitForShoot, ()->{
                    System.out.println("Transitioned from increment3 to wait for shoot because bad intake detected");
                })
                .transition(()->shooterButton, States.WaitForShoot, ()->{
                    System.out.println("Transitioned from increment3 to wait for shoot because shooter button pressed");
                })

                .state(States.WaitForShoot)
                .loop(()->{
                    spindexer.setPosition(Spindexer.SpindexerPosition.Shoot2);
                    intakes.setBadIntakePower(0.3);
                    intakes.setGoodIntakePower(1);

                    shooter.setUpperGateOpen(true);
                })
                .transition(()->shooterButton, States.Kick1)

                .state(States.Kick1)
                .onEnter(()->{
                    shooterButton=false;
                    spindexer.setPosition(Spindexer.SpindexerPosition.Shoot2);
                    spindexer.setLowerGateOpen(true);
                    shooter.setUpperGateOpen(true);
                    spindexer.setKickerPos(true);
                })
                .transitionTimed(shootWaitTime, States.ShootSpin1)

                .state(States.ShootSpin1)
                .onEnter(()->{
                    spindexer.setPosition(Spindexer.SpindexerPosition.Shoot1);
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
                    spindexer.setPosition(Spindexer.SpindexerPosition.Shoot0);
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
                .transitionTimed(0.2, States.Have3WaitShoot)

                .state(States.Have3WaitShoot)
                .onEnter(()->{
                    spindexer.setKickerPos(true);
                })
                .transitionTimed(1.2, States.Intake)
                .build();


        StateMachine autoMachine = new StateMachineBuilder() //Autonomia
                .transition(()->follower.atParametricEnd())
                .transitionTimed(0.4)
                .state(AutoStates.MOVETOSHOOT1)
                .onEnter(()->{
                    PathChain toScore = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), shootPose))
                            .setLinearHeadingInterpolation(follower.getHeading(),shootPose.getHeading())
                            .setBrakingStrength(3)
                            .build();
                    follower.followPath(toScore, true);
                    shooter.setHood(0.67);
                    shooter.setTargetVelocity(1940);
                    shooter.setTurretPos(shooter.convertDegreestoServoPos(-108*Posmultiplier));
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(0.5)

                .state(AutoStates.wait1)
                .transition(()->Math.abs(shooter.getTargetVelo()-shooter.getCurrentVelocity())<20)
                .transitionTimed(2.5)

                .state(AutoStates.SHOOT1)
                .onEnter(()->{
                    shooterButton=true;
                })
                .transition(()->stateMachine.getState()== States.Intake)

                .state(AutoStates.MOVETOINTAKE1)
                .onEnter(()->{
                    shooter.setTurretPos(shooter.convertDegreestoServoPos(-107 *Posmultiplier));

                    PathChain toIntake = follower.pathBuilder()
                            .addPath(new BezierCurve(follower.getPose(), intake1Pose, intake1donePose))
                            .setLinearHeadingInterpolation(follower.getHeading(),intake1donePose.getHeading())
                            .setBrakingStrength(3)
                            .build();
                    follower.followPath(toIntake, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.4)
                .transition(()->stateMachine.getState()== States.WaitForShoot)

                .state(AutoStates.Out1)
                .onEnter(()->{
                    PathChain toIntakeback = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), intake1Pose))
                            .setLinearHeadingInterpolation(follower.getHeading(),intake1donePose.getHeading())
                            .setBrakingStrength(3)
                            .setNoDeceleration()
                            .build();
                    follower.followPath(toIntakeback, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(0.3)
                .transition(()->stateMachine.getState()== States.WaitForShoot)

                .state(AutoStates.INTAKE1)
                .onEnter(()->{

                    PathChain toIntake = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), intake1donePose))
                            .setLinearHeadingInterpolation(follower.getHeading(),intake1donePose.getHeading())
                            .setBrakingStrength(3)
                            .build();
                    follower.followPath(toIntake, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1)
                .transition(()->stateMachine.getState()== States.WaitForShoot)

                .state(AutoStates.MOVETOSHOOT2)
                .onEnter(()->{
                    PathChain toScore = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), shootPose))
                            .setLinearHeadingInterpolation(follower.getHeading(),shootPose.getHeading())
                            .setBrakingStrength(3)
                            .build();
                    follower.followPath(toScore, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(2.5)

                .state(AutoStates.wait2)
                .transitionTimed(0.2)

                .state(AutoStates.SHOOT2)
                .onEnter(()->{
                    shooterButton=true;

                })
                .transition(()->stateMachine.getState()== States.Intake)
                .state(AutoStates.MOVETOINTAKE2)
                .onEnter(()->{
                    shooter.setTurretPos(shooter.convertDegreestoServoPos(-108*Posmultiplier));

                    PathChain toIntakeHuman = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), intakeHuman))
                            .setLinearHeadingInterpolation(follower.getHeading(),intakeHuman.getHeading())
                            .setNoDeceleration()
                            .build();
                    follower.followPath(toIntakeHuman, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.3)
                .transition(()->stateMachine.getState()== States.WaitForShoot)

                .state(AutoStates.Out2)
                .onEnter(()->{
                    PathChain toIntakeHuman = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), intakeHumanOut))
                            .setLinearHeadingInterpolation(follower.getHeading(),intakeHumanOut.getHeading())
                            .setNoDeceleration()
                            .build();
                    follower.followPath(toIntakeHuman, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(0.3)
                .transition(()->stateMachine.getState()== States.WaitForShoot)

                .state(AutoStates.INTAKE2)
                .onEnter(()->{
                    PathChain toIntakeHuman = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), intakeHuman))
                            .setLinearHeadingInterpolation(follower.getHeading(),intakeHuman.getHeading())
                            .setNoDeceleration()
                            .build();
                    follower.followPath(toIntakeHuman, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(0.6)
                .transition(()->stateMachine.getState()== States.WaitForShoot)

                .state(AutoStates.MOVETOSHOOT3)
                .onEnter(()->{
                    PathChain toScore = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), shootPose))
                            .setLinearHeadingInterpolation(follower.getHeading(),shootPose.getHeading())
                            .setBrakingStrength(3)
                            .build();
                    follower.followPath(toScore, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(2.5)

                .state(AutoStates.wait3)
                .transitionTimed(0.2)

                .state(AutoStates.SHOOT3)
                .onEnter(()->{
                    shooterButton=true;

                })




                ////////////////////////////////////////////////////////////AUTODETECTS





                .transition(()->stateMachine.getState()== States.Intake)
                .state(AutoStates.MOVETOINTAKE3)
                .onEnter(()->{
                    LLFieldScannerResults results = limelightCamera.getTrackingResults();
                    if (results != null) {
                        double dx = results.getPosition()[1];

                        if (Posmultiplier == 1) {
                            dx = Math.max(dx, -9);
                        } else {
                            dx = Math.min(dx, 9);
                        }
                        Pose intakePoseAuto = new Pose(follower.getPose().getX() - Posmultiplier * dx, -63 * Posmultiplier, Math.toRadians(90 * Posmultiplier));
                        PathChain toIntake = follower.pathBuilder()
                                .addPath(new BezierLine(follower.getPose(), intakePoseAuto))
                                .setLinearHeadingInterpolation(follower.getHeading(), intakePoseAuto.getHeading())
                                .build();
                        follower.followPath(toIntake, true);
                        System.out.println("Detected at " + dx);

                    }else{
                        PathChain toIntake = follower.pathBuilder()
                                .addPath(new BezierLine(follower.getPose(), intakeHuman))
                                .setLinearHeadingInterpolation(follower.getHeading(),intakeHuman.getHeading())
                                .build();
                        follower.followPath(toIntake, true);
                        System.out.println("Detected at NULL");

                    }

                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.3)
                .transition(()->stateMachine.getState()== States.WaitForShoot)
                .state(AutoStates.Out3)
                .onEnter(()->{
                    Pose intakePoseOut = new Pose(follower.getPose().getX(), -57*Posmultiplier, Math.toRadians(90*Posmultiplier));
                    PathChain toIntakeout = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), intakePoseOut))
                            .setLinearHeadingInterpolation(follower.getHeading(),intakePoseOut.getHeading())
                            .setNoDeceleration()
                            .build();
                    follower.followPath(toIntakeout, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(0.4)
                .transition(()->stateMachine.getState()== States.WaitForShoot)

                .state(AutoStates.INTAKE3)
                .onEnter(()->{
                    Pose intakePoseBackIn = new Pose(follower.getPose().getX(), -63*Posmultiplier, Math.toRadians(90*Posmultiplier));
                    PathChain toIntake = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), intakePoseBackIn))
                            .setLinearHeadingInterpolation(follower.getHeading(),intakePoseBackIn.getHeading())
                            .build();
                    follower.followPath(toIntake, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(0.6)
                .transition(()->stateMachine.getState()== States.WaitForShoot)

                .state(AutoStates.MOVETOSHOOT4)
                .onEnter(()->{

                    PathChain toScore = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), shootPose))
                            .setLinearHeadingInterpolation(follower.getHeading(),shootPose.getHeading())
                            .setBrakingStrength(3)
                            .build();
                    follower.followPath(toScore, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(2.5)

                .state(AutoStates.wait4)
                .transitionTimed(0.2)

                .state(AutoStates.SHOOT4)
                .onEnter(()->{
                    shooterButton=true;

                })
                .transition(()->stateMachine.getState()== States.Intake)
                .transitionTimed(1.67)




                .state(AutoStates.MOVETOINTAKE4)
                .onEnter(()->{
                    LLFieldScannerResults results = limelightCamera.getTrackingResults();
                    if (results != null) {
                        double dx = results.getPosition()[1];

                        if (Posmultiplier == 1) {
                            dx = Math.max(dx, -9);
                        } else {
                            dx = Math.min(dx, 9);
                        }
                        Pose intakePoseAuto = new Pose(follower.getPose().getX() - Posmultiplier * dx, -63 * Posmultiplier, Math.toRadians(90 * Posmultiplier));
                        PathChain toIntake = follower.pathBuilder()
                                .addPath(new BezierLine(follower.getPose(), intakePoseAuto))
                                .setLinearHeadingInterpolation(follower.getHeading(), intakePoseAuto.getHeading())
                                .build();
                        follower.followPath(toIntake, true);
                        System.out.println("Detected at " + dx);

                    }else{
                        PathChain toIntake = follower.pathBuilder()
                                .addPath(new BezierLine(follower.getPose(), intakeHuman))
                                .setLinearHeadingInterpolation(follower.getHeading(),intakeHuman.getHeading())
                                .build();
                        follower.followPath(toIntake, true);
                        System.out.println("Detected at NULL");

                    }

                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.3)
                .transition(()->stateMachine.getState()== States.WaitForShoot)
                .state(AutoStates.Out4)
                .onEnter(()->{
                    Pose intakePoseOut = new Pose(follower.getPose().getX(), -57*Posmultiplier, Math.toRadians(90*Posmultiplier));
                    PathChain toIntakeout = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), intakePoseOut))
                            .setLinearHeadingInterpolation(follower.getHeading(),intakePoseOut.getHeading())
                            .setNoDeceleration()
                            .build();
                    follower.followPath(toIntakeout, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(0.4)
                .transition(()->stateMachine.getState()== States.WaitForShoot)

                .state(AutoStates.INTAKE4)
                .onEnter(()->{
                    Pose intakePoseBackIn = new Pose(follower.getPose().getX(), -63*Posmultiplier, Math.toRadians(90*Posmultiplier));
                    PathChain toIntake = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), intakePoseBackIn))
                            .setLinearHeadingInterpolation(follower.getHeading(),intakePoseBackIn.getHeading())
                            .build();
                    follower.followPath(toIntake, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(0.6)
                .transition(()->stateMachine.getState()== States.WaitForShoot)

                .state(AutoStates.MOVETOSHOOT5)
                .onEnter(()->{

                    PathChain toScore = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), shootPose))
                            .setLinearHeadingInterpolation(follower.getHeading(),shootPose.getHeading())
                            .setBrakingStrength(3)
                            .build();
                    follower.followPath(toScore, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(2.5)

                .state(AutoStates.wait5)
                .transitionTimed(0.2)

                .state(AutoStates.SHOOT5)
                .onEnter(()->{
                    shooterButton=true;

                })
                .transition(()->stateMachine.getState()== States.Intake)
                .transitionTimed(1.67)



                .state(AutoStates.MOVETOINTAKE5)
                .onEnter(()->{
                    LLFieldScannerResults results = limelightCamera.getTrackingResults();
                    if (results != null) {
                        double dx = results.getPosition()[1];

                        if (Posmultiplier == 1) {
                            dx = Math.max(dx, -9);
                        } else {
                            dx = Math.min(dx, 9);
                        }
                        Pose intakePoseAuto = new Pose(follower.getPose().getX() - Posmultiplier * dx, -63 * Posmultiplier, Math.toRadians(90 * Posmultiplier));
                        PathChain toIntake = follower.pathBuilder()
                                .addPath(new BezierLine(follower.getPose(), intakePoseAuto))
                                .setLinearHeadingInterpolation(follower.getHeading(), intakePoseAuto.getHeading())
                                .build();
                        follower.followPath(toIntake, true);
                        System.out.println("Detected at " + dx);

                    }else{
                        PathChain toIntake = follower.pathBuilder()
                                .addPath(new BezierLine(follower.getPose(), intakeHuman))
                                .setLinearHeadingInterpolation(follower.getHeading(),intakeHuman.getHeading())
                                .build();
                        follower.followPath(toIntake, true);
                        System.out.println("Detected at NULL");

                    }

                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.3)
                .transition(()->stateMachine.getState()== States.WaitForShoot)
                .state(AutoStates.Out5)
                .onEnter(()->{
                    Pose intakePoseOut = new Pose(follower.getPose().getX(), -57*Posmultiplier, Math.toRadians(90*Posmultiplier));
                    PathChain toIntakeout = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), intakePoseOut))
                            .setLinearHeadingInterpolation(follower.getHeading(),intakePoseOut.getHeading())
                            .setNoDeceleration()
                            .build();
                    follower.followPath(toIntakeout, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(0.4)
                .transition(()->stateMachine.getState()== States.WaitForShoot)

                .state(AutoStates.INTAKE5)
                .onEnter(()->{
                    Pose intakePoseBackIn = new Pose(follower.getPose().getX(), -63*Posmultiplier, Math.toRadians(90*Posmultiplier));
                    PathChain toIntake = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), intakePoseBackIn))
                            .setLinearHeadingInterpolation(follower.getHeading(),intakePoseBackIn.getHeading())
                            .build();
                    follower.followPath(toIntake, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(0.6)
                .transition(()->stateMachine.getState()== States.WaitForShoot)

                .state(AutoStates.MOVETOSHOOT6)
                .onEnter(()->{

                    PathChain toScore = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), shootPose))
                            .setLinearHeadingInterpolation(follower.getHeading(),shootPose.getHeading())
                            .setBrakingStrength(3)
                            .build();
                    follower.followPath(toScore, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(2.5)

                .state(AutoStates.wait6)
                .transitionTimed(0.2)

                .state(AutoStates.SHOOT6)
                .onEnter(()->{
                    shooterButton=true;

                })
                .transition(()->stateMachine.getState()== States.Intake)
                .transitionTimed(1.67)


                .state(AutoStates.park)
                .onEnter(()->{
                    PathChain park = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), leave))
                            .setLinearHeadingInterpolation(follower.getHeading(),leave.getHeading())
                            .setBrakingStrength(0.6)
                            .build();
                    follower.followPath(park, true);
                })
                .build();

        stateMachine.start();
        autoMachine.start();
        limelightCamera.setCurrentPipeline(LimelightCamera.Pipelines.BALLTRACKING);
        stateMachine.setState(States.WaitForShoot);
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

