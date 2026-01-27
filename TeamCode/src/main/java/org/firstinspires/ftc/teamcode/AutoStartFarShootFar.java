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
@Autonomous(name = "AutoFarFar", group = "Auto")
public class AutoStartFarShootFar extends LinearOpMode {
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
    public double shootWaitTime = 0.23;
    public static boolean forceWait = false;
    Runnable setForcewaitTrue = () -> forceWait = true;


    public static Shooter.Goal shooterTarget = Shooter.Goal.BLUE;


    public enum AutoStates {
        MOVETOSHOOT1, wait1, SHOOT1,
        MOVETOINTAKE1, INTAKE1,
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
        Shoot,
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


        Pose startPose = new Pose(60, -15*Posmultiplier, Math.toRadians(90*Posmultiplier));
        Pose shootPose = new Pose(56, -12*Posmultiplier, Math.toRadians(90*Posmultiplier));

        Pose intakeHuman = new Pose(59, -57*Posmultiplier, Math.toRadians(90*Posmultiplier));
        Pose intakeHumanOut = new Pose(59, -50*Posmultiplier, Math.toRadians(90*Posmultiplier));
        Pose intake1Pose = new Pose(31, -17*Posmultiplier, Math.toRadians(90*Posmultiplier));

        Pose intake1donePose = new Pose(33, -70*Posmultiplier, Math.toRadians(90*Posmultiplier));
        Pose leave = new Pose(40, -30*Posmultiplier, Math.toRadians(90*Posmultiplier));



        follower.setStartingPose(startPose);


        StateMachine stateMachine = new StateMachineBuilder()
                .state(States.Intake)
                .loop(()->{
                    shooter.setUpperGateOpen(false);
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
                .transition(()->forceWait, States.WaitForShoot, ()->{
                    System.out.println("Transitioned from intake to wait for shoot because force wait true");
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
                .transition(()->forceWait, States.WaitForShoot, ()->{
                    System.out.println("Transitioned from wait1 to wait for shoot because force wait true");
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
                .transition(()->forceWait, States.WaitForShoot, ()->{
                    System.out.println("Transitioned from increment2 to wait for shoot because force wait true");
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
                .transition(()->forceWait, States.WaitForShoot, ()->{
                    System.out.println("Transitioned from wait2 to wait for shoot because force wait true");
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
                .transition(()->forceWait, States.WaitForShoot, ()->{
                    System.out.println("Transitioned from increment3 to wait for shoot because force wait true");
                })


                .state(States.WaitForShoot)
                .loop(()->{
                    if (shootorder[0]==0){
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot1);
                    }else if (shootorder[0]==1){
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot2);
                    }else{
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot0);
                    }
                    intakes.setBadIntakePower(0.3);
                    intakes.setGoodIntakePower(1);

                    shooter.setUpperGateOpen(true);
                    forceWait = false;
                })
                .transition(()->shooterButton, States.Kick1)

                .state(States.Kick1)
                .onEnter(()->{
                    forceWait = false;
                    shooterButton=false;
                    if (shootorder[0]==0){
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot1);
                    }else if (shootorder[0]==1){
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot2);
                    }else{
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot0);
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
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot4);
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
                    shooter.setTargetVelocity(2000);
                    shooter.setTurretPos(shooter.convertDegreestoServoPos(-140*Posmultiplier));
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(0.5)

                .state(AutoStates.wait1)
                .transitionTimed(2)

                .state(AutoStates.SHOOT1)
                .onEnter(()->{
                    shooterButton=true;
                })
                .transitionTimed(1.3)

                .state(AutoStates.MOVETOINTAKE1)
                .onEnter(()->{
                    PathChain toIntake = follower.pathBuilder()
                            .addPath(new BezierCurve(follower.getPose(), intake1Pose, intake1donePose))
                            .setTangentHeadingInterpolation()
                            .setBrakingStrength(3)
                            .build();
                    follower.followPath(toIntake, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(2.4)

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
                .transitionTimed(1.5)
                .state(AutoStates.MOVETOINTAKE2)
                .onEnter(()->{
                    PathChain toIntakeHuman = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), intakeHuman))
                            .setLinearHeadingInterpolation(follower.getHeading(),intakeHuman.getHeading())
                            .setBrakingStrength(3)
                            .build();
                    follower.followPath(toIntakeHuman, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.3)

                .state(AutoStates.Out2)
                .onEnter(()->{
                    PathChain toIntakeHuman = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), intakeHumanOut))
                            .setLinearHeadingInterpolation(follower.getHeading(),intakeHumanOut.getHeading())
                            .setBrakingStrength(0.6)
                            .build();
                    follower.followPath(toIntakeHuman, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(0.3)
                .state(AutoStates.INTAKE2)
                .onEnter(()->{
                    PathChain toIntakeHuman = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), intakeHuman))
                            .setLinearHeadingInterpolation(follower.getHeading(),intakeHuman.getHeading())
                            .setBrakingStrength(3)
                            .build();
                    follower.followPath(toIntakeHuman, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(0.6)
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
                .transitionTimed(1.5)
                .state(AutoStates.MOVETOINTAKE3)
                .onEnter(()->{
                    PathChain toIntakeHuman = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), intake1donePose))
                            .setLinearHeadingInterpolation(follower.getHeading(),intake1donePose.getHeading())
                            .setBrakingStrength(3)
                            .build();
                    follower.followPath(toIntakeHuman, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.3)

                .state(AutoStates.Out3)
                .onEnter(()->{
                    PathChain toIntakeout = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), intake1Pose))
                            .setLinearHeadingInterpolation(follower.getHeading(),intake1Pose.getHeading())
                            .setBrakingStrength(0.6)
                            .build();
                    follower.followPath(toIntakeout, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(0.2)
                .state(AutoStates.INTAKE3)
                .onEnter(()->{
                    PathChain toIntake = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), intake1donePose))
                            .setLinearHeadingInterpolation(follower.getHeading(),intake1donePose.getHeading())
                            .setBrakingStrength(3)
                            .build();
                    follower.followPath(toIntake, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(0.6)
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
                .transitionTimed(1.5)
                .state(AutoStates.MOVETOINTAKE4)
                .onEnter(()->{
                    PathChain toIntakeHuman = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), intakeHuman))
                            .setLinearHeadingInterpolation(follower.getHeading(),intakeHuman.getHeading())
                            .setBrakingStrength(3)
                            .build();
                    follower.followPath(toIntakeHuman, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.3)

                .state(AutoStates.Out4)
                .onEnter(()->{
                    PathChain toIntakeHuman = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), intakeHumanOut))
                            .setLinearHeadingInterpolation(follower.getHeading(),intakeHumanOut.getHeading())
                            .setBrakingStrength(0.6)
                            .build();
                    follower.followPath(toIntakeHuman, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(0.3)
                .state(AutoStates.INTAKE4)
                .onEnter(()->{
                    PathChain toIntakeHuman = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), intakeHuman))
                            .setLinearHeadingInterpolation(follower.getHeading(),intakeHuman.getHeading())
                            .setBrakingStrength(3)
                            .build();
                    follower.followPath(toIntakeHuman, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(0.6)
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
                .transitionTimed(1.5)
                .state(AutoStates.MOVETOINTAKE5)
                .onEnter(()->{
                    PathChain toIntakeHuman = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), intake1donePose))
                            .setLinearHeadingInterpolation(follower.getHeading(),intake1donePose.getHeading())
                            .setBrakingStrength(3)
                            .build();
                    follower.followPath(toIntakeHuman, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(1.3)

                .state(AutoStates.Out5)
                .onEnter(()->{
                    PathChain toIntakeout = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), intake1Pose))
                            .setLinearHeadingInterpolation(follower.getHeading(),intake1Pose.getHeading())
                            .setBrakingStrength(0.6)
                            .build();
                    follower.followPath(toIntakeout, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(0.2)
                .state(AutoStates.INTAKE5)
                .onEnter(()->{
                    PathChain toIntake = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), intake1donePose))
                            .setLinearHeadingInterpolation(follower.getHeading(),intake1donePose.getHeading())
                            .setBrakingStrength(3)
                            .build();
                    follower.followPath(toIntake, true);
                })
                .transition(()->follower.atParametricEnd())
                .transitionTimed(0.6)
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
                .transitionTimed(1.5)
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

