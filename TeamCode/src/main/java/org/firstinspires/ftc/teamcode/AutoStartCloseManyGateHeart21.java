package org.firstinspires.ftc.teamcode;


//import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower;

import static org.firstinspires.ftc.teamcode.pedro.Constants.createFollower;

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

//import org.firstinspires.ftc.teamcode.pedroPathing.PositionLogger;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
//import org.firstinspires.ftc.teamcode.subsystems.Intakes;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;
//import org.firstinspires.ftc.teamcode.subsystems.Position;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
//import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

import java.io.IOException;
import java.util.List;

@Configurable
@Autonomous(name = "HeartClose21SensorRED", group = "Auto")
public class AutoStartCloseManyGateHeart21 extends LinearOpMode {
    private Follower follower;
    Intake intakes;
    Shooter shooter;
    Kicker spindexer;
    public int pattern = 1;
    public boolean shooterButton = false;
    public static Shooter.Goal shooterTarget = Shooter.Goal.RED;
//    PositionLogger positionLogger;


    public enum AutoStates {
        MOVETOSHOOT1, wait1, SHOOT1,
        MOVETOINTAKE1,
        MOVETOSHOOT2, wait2, SHOOT2,
        MOVETOINTAKE2, closegate2, back2, gate2,
        MOVETOSHOOT3, wait3, SHOOT3,
        MOVETOINTAKE3, closegate3,
        MOVETOSHOOT4, wait4, SHOOT4,
        MOVETOINTAKE4,closegate4,
        MOVETOSHOOT5, wait5, SHOOT5,
        MOVETOINTAKE5,closegate5,
        MOVETOSHOOT6, wait6, SHOOT6,
        MOVETOINTAKE6,closegate6,
        MOVETOSHOOT7, wait7, SHOOT7,
        MOVETOINTAKE7,closegate7,
        MOVETOSHOOT8, wait8, SHOOT8
    }

    public enum States {
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
        shooterTarget = Shooter.Goal.RED;
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs)
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        intakes = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        spindexer = new Kicker(hardwareMap);
        follower = createFollower(hardwareMap);

        while (opModeInInit()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();
            follower.update();
            shooter.setUpperGate(false);
            shooter.setTurretPos(shooter.convertDegreestoServoPos(0));
            shooter.setHood(0.67);

            telemetry.addData("Pattern", pattern);
            telemetry.addData("Init Pose: ", follower.getPose());
            telemetry.addLine("ALLIANCE: RED");

            telemetry.update();
            spindexer.update();
            shooter.update();
        }

        waitForStart();


        Pose gateopen = new Pose(-3, 59, Math.toRadians(90));
        Pose gatepreopen2ndtime = new Pose(4, 30, Math.toRadians(90));


        Pose gateintake1 = new Pose(7, 63, Math.toRadians(110));
        Pose gateintake2 = new Pose(6.5, 63, Math.toRadians(110));
        Pose gateintake3 = new Pose(6.5, 63, Math.toRadians(110));
        Pose gateintake4 = new Pose(5, 63, Math.toRadians(110));

        Pose gateintakecontrol = new Pose(8, 42, Math.toRadians(-77));

        Pose startPose = new Pose(-63.3, 40, Math.toRadians(0));

        Pose shootPose = new Pose(-32, 16, Math.toRadians(40));
        Pose shootPose2nd = new Pose(-17, 17, Math.toRadians(50));
        Pose intake1Pose = new Pose(-17, 27, Math.toRadians(80));
        Pose intake2Pose = new Pose(13, 29, Math.toRadians(80));

        Pose intake1donePose = new Pose(-16, 58, Math.toRadians(90));
        Pose intake2donePose = new Pose(10, 63, Math.toRadians(90));

        PathChain toShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.heading(), shootPose.heading())
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

        PathChain openGate2 = follower.pathBuilder()
                .addPath(new BezierLine(gatepreopen2ndtime, gateopen))
                .setLinearHeadingInterpolation(gatepreopen2ndtime.heading(), gateopen.heading())
                .build();

        PathChain intake2back = follower.pathBuilder()
                .addPath(new BezierCurve(intake2donePose, intake2Pose, gatepreopen2ndtime))
                .setNoDeceleration()
                .setLinearHeadingInterpolation(intake2donePose.heading(), gatepreopen2ndtime.heading())
                .build();


        PathChain toScore1 = follower.pathBuilder()
                .addPath(new BezierLine(intake1donePose, shootPose2nd))
                .setLinearHeadingInterpolation(intake1donePose.heading(), shootPose2nd.heading())
                .setBrakingStrength(1)
                .build();

        PathChain toScore2 = follower.pathBuilder()
                .addPath(new BezierLine(gateopen, shootPose2nd))
                .setBrakingStrength(1)
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();


        follower.setStartingPose(startPose);


        StateMachine autoMachine = new StateMachineBuilder() //Autonomia
                .state(AutoStates.MOVETOSHOOT1)
                .onEnter(() -> {
                    shooter.setUpperGate(false);
                    follower.followPath(toShoot, true);
                    shooter.setHood(0.6);
                    intakes.setPower(0.3);
                    shooter.setTargetVelocity(1320);
                    shooter.setTurretPos(shooter.convertDegreestoServoPos(-86));
                })

                .transitionTimed(1.7)
                .state(AutoStates.wait1)
                .onEnter(() -> {
                    intakes.setPower(1);
                    spindexer.setKicker(true);

                    shooter.setUpperGate(true);

                })
                .transitionTimed(0.1)
                .state(AutoStates.SHOOT1)
                .onEnter(() -> {
                })
                .transitionTimed(0.3)
                .state(AutoStates.MOVETOINTAKE1)
                .onEnter(() -> {
                    shooter.setUpperGate(false);
                    spindexer.setKicker(false);
                    follower.followPath(toIntake1, true);
                })

                .transition(() -> follower.atParametricEnd())
                .transition(() -> follower.getCurrentTValue() > 0.82).transitionTimed(1)

                .state(AutoStates.MOVETOSHOOT2)
                .onEnter(() -> {
                    intakes.setPower(1);
                    shooter.setTurretPos(shooter.convertDegreestoServoPos(-97));
                    shooter.setHood(0.67);
                    shooter.setTargetVelocity(1380);
                    follower.followPath(toScore1, true);
                })
                .transitionTimed(1.5)

                .state(AutoStates.wait2)
                .onEnter(() -> {
                    shooter.setUpperGate(true);
                })
                .transitionTimed(0.1)
                .state(AutoStates.SHOOT2)
                .onEnter(() -> {
                    spindexer.setKicker(true);
                })
                .transitionTimed(0.2)
                .state(AutoStates.MOVETOINTAKE2)
                .onEnter(() -> {
                    spindexer.setKicker(false);
                    follower.followPath(toIntake2, true);
                })
                .transitionTimed(0.15)
                .state(AutoStates.closegate2)
                .onEnter(() -> {
                    shooter.setUpperGate(false);
                })
                .transition(() -> follower.atParametricEnd())
                .transition(() -> follower.getCurrentTValue() > 0.82)
                .transitionTimed(0.95)
                .transition(() -> intakes.getBeamBreakOutside() && intakes.getBeamBreakInside() && intakes.getDetected())

                .state(AutoStates.back2)
                .onEnter(() -> {
                    follower.followPath(intake2back, true);
                })

                .transitionTimed(0.3)
                .state(AutoStates.gate2)
                .onEnter(() -> {
                    intakes.setPower(0);
                    follower.followPath(openGate2, true);
                })

                .transitionTimed(0.5)
                .state(AutoStates.MOVETOSHOOT3)
                .onEnter(() -> {
                    intakes.setPower(1);
                    shooter.setTurretPos(shooter.convertDegreestoServoPos(-65));
                    shooter.setHood(0.66);
                    shooter.setTargetVelocity(1370);

                    follower.followPath(toScore2, true);
                })
                .transitionTimed(1.46)

                .state(AutoStates.wait3)
                .onEnter(() -> {
                    shooter.setUpperGate(true);
                })
                .transitionTimed(0.1)
                .state(AutoStates.SHOOT3)
                .onEnter(() -> {
                    spindexer.setKicker(true);
                })
                .transitionTimed(0.1)
                .state(AutoStates.MOVETOINTAKE3)
                .onEnter(() -> {
                    spindexer.setKicker(false);
                    PathChain toIntakeGate = follower.pathBuilder()
                            .addPath(new BezierCurve(follower.getPose(), gateintakecontrol, gateintake1))
                            .setLinearHeadingInterpolation(follower.heading(), gateintake1.heading())
                            .setBrakingStrength(0.4)
                            .build();
                    follower.followPath(toIntakeGate, true);
                })
                .transitionTimed(0.15)
                .state(AutoStates.closegate3)
                .onEnter(() -> {
                    shooter.setUpperGate(false);
                })
                .transition(() -> intakes.getBeamBreakOutside() && intakes.getBeamBreakInside() && intakes.getDetected())
                .transitionTimed(2.35)
                /*
                .state(AutoStates.backupgate1)
                .onEnter(()->{
                    PathChain back = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), gateintakeback1))
                            .setLinearHeadingInterpolation(follower.heading(), gateintakeback1.heading())
                            .setBrakingStrength(0.4)
                            .build();
                    follower.followPath(back, true);
                })
                .transitionTimed(0.4)
                .state(AutoStates.upgate1)
                .onEnter(()->{
                    PathChain up = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), gateintake1))
                            .setLinearHeadingInterpolation(follower.heading(), gateintake1.heading())
                            .setBrakingStrength(0.4)
                            .build();
                    follower.followPath(up, true);
                })
                .transitionTimed(0.4)
*/
                .state(AutoStates.MOVETOSHOOT4)
                .onEnter(() -> {
                    shooter.setTurretPos(shooter.convertDegreestoServoPos(-81));
                    shooter.setHood(0.65);
                    shooter.setTargetVelocity(1370);
                    PathChain toScoreGate = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), shootPose2nd))
                            .setReversed()
                            .setBrakingStrength(0.6)
                            .build();
                    follower.followPath(toScoreGate, false);
                })

                .transition(() -> follower.atParametricEnd())
                .transition(() -> follower.getCurrentTValue() > 0.82).transitionTimed(1.5)
                .state(AutoStates.wait4)
                .onEnter(() -> {
                    follower.pausePathFollowing();
                    intakes.setPower(1);
                    shooter.setUpperGate(true);
                })
                .transitionTimed(0.1)
                .state(AutoStates.SHOOT4)
                .onEnter(() -> {
                    spindexer.setKicker(true);
                })
                .transitionTimed(0.3)


                .state(AutoStates.MOVETOINTAKE4)
                .onEnter(() -> {
                    spindexer.setKicker(false);
                    PathChain toIntakeGate = follower.pathBuilder()
                            .addPath(new BezierCurve(follower.getPose(), gateintakecontrol, gateintake2))
                            .setLinearHeadingInterpolation(follower.heading(), gateintake2.heading())
                            .setBrakingStrength(0.6)
                            .build();
                    follower.followPath(toIntakeGate, true);
                })
                .transitionTimed(0.15)
                .state(AutoStates.closegate4)
                .onEnter(() -> {
                    shooter.setUpperGate(false);
                })
                .transition(() -> intakes.getBeamBreakOutside() && intakes.getBeamBreakInside() && intakes.getDetected())
                .transitionTimed(2.35)
                /*
                .state(AutoStates.backupgate2)
                .onEnter(()->{
                    PathChain back = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), gateintakeback2))
                            .setLinearHeadingInterpolation(follower.heading(), gateintakeback2.heading())
                            .setBrakingStrength(0.4)
                            .build();
                    follower.followPath(back, true);
                })
                .transitionTimed(0.4)
                .state(AutoStates.upgate2)
                .onEnter(()->{
                    PathChain up = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), gateintake2))
                            .setLinearHeadingInterpolation(follower.heading(), gateintake2.heading())
                            .setBrakingStrength(0.4)
                            .build();
                    follower.followPath(up, true);
                })
                .transitionTimed(0.4)

                 */

                .state(AutoStates.MOVETOSHOOT5)
                .onEnter(() -> {
                    shooter.setTurretPos(shooter.convertDegreestoServoPos(-78));
                    shooter.setHood(0.67);
                    shooter.setTargetVelocity(1360);
                    PathChain toScoreGate = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), shootPose2nd))
                            .setReversed()
                            .setBrakingStrength(0.6)
                            .build();
                    follower.followPath(toScoreGate, false);
                })

                .transition(() -> follower.atParametricEnd())
                .transition(() -> follower.getCurrentTValue() > 0.82).transitionTimed(1.4)
                .state(AutoStates.wait5)
                .onEnter(() -> {
                    follower.pausePathFollowing();
                    intakes.setPower(1);
                    shooter.setUpperGate(true);
                })
                .transitionTimed(0.1)
                .state(AutoStates.SHOOT5)
                .onEnter(() -> {
                    spindexer.setKicker(true);
                })
                .transitionTimed(0.3)

                .state(AutoStates.MOVETOINTAKE5)
                .onEnter(() -> {
                    spindexer.setKicker(false);
                    PathChain toIntakeGate = follower.pathBuilder()
                            .addPath(new BezierCurve(follower.getPose(), gateintakecontrol, gateintake3))
                            .setLinearHeadingInterpolation(follower.heading(), gateintake3.heading())
                            .setBrakingStrength(0.6)
                            .build();
                    follower.followPath(toIntakeGate, true);
                })
                .transitionTimed(0.15)
                .state(AutoStates.closegate5)
                .onEnter(() -> {
                    shooter.setUpperGate(false);
                })
                .transition(() -> intakes.getBeamBreakOutside() && intakes.getBeamBreakInside() && intakes.getDetected())
                .transitionTimed(1.95)
                /*
                .state(AutoStates.backupgate3)
                .onEnter(()->{
                    PathChain back = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), gateintakeback3))
                            .setLinearHeadingInterpolation(follower.heading(), gateintakeback3.heading())
                            .setBrakingStrength(0.4)
                            .build();
                    follower.followPath(back, true);
                })
                                .transitionTimed(0.4)
                .state(AutoStates.upgate3)
                .onEnter(()->{
                    PathChain up = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), gateintake3))
                            .setLinearHeadingInterpolation(follower.heading(), gateintake3.heading())
                            .setBrakingStrength(0.4)
                            .build();
                    follower.followPath(up, true);
                })
                .transitionTimed(0.4)

                 */

                .state(AutoStates.MOVETOSHOOT6)
                .onEnter(() -> {
                    shooter.setTurretPos(shooter.convertDegreestoServoPos(-76));
                    shooter.setHood(0.68);
                    shooter.setTargetVelocity(1390);
                    PathChain toScoreGate = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), shootPose2nd))
                            .setReversed()
                            .setBrakingStrength(0.6)
                            .build();
                    follower.followPath(toScoreGate, false);
                })

                .transition(() -> follower.atParametricEnd())
                .transition(() -> follower.getCurrentTValue() > 0.82).transitionTimed(1.6)
                .state(AutoStates.wait6)
                .onEnter(() -> {
                    follower.pausePathFollowing();
                    intakes.setPower(1);
                    shooter.setUpperGate(true);
                })
                .transitionTimed(0.1)
                .state(AutoStates.SHOOT6)
                .onEnter(() -> {
                    spindexer.setKicker(true);
                })
                .transitionTimed(0.3)
                .state(AutoStates.MOVETOINTAKE6)
                .onEnter(() -> {
                    spindexer.setKicker(false);
                    PathChain toIntakeGate = follower.pathBuilder()
                            .addPath(new BezierCurve(follower.getPose(), gateintakecontrol, gateintake4))
                            .setLinearHeadingInterpolation(follower.heading(), gateintake4.heading())
                            .setBrakingStrength(0.6)
                            .build();
                    follower.followPath(toIntakeGate, true);
                })
                .transitionTimed(0.15)
                .state(AutoStates.closegate6)
                .onEnter(() -> {
                    shooter.setUpperGate(false);
                })
                .transition(() -> intakes.getBeamBreakOutside() && intakes.getBeamBreakInside() && intakes.getDetected())
                .transitionTimed(2.35)
                /*
                .state(AutoStates.backupgate3)
                .onEnter(()->{
                    PathChain back = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), gateintakeback3))
                            .setLinearHeadingInterpolation(follower.heading(), gateintakeback3.heading())
                            .setBrakingStrength(0.4)
                            .build();
                    follower.followPath(back, true);
                })
                                .transitionTimed(0.4)
                .state(AutoStates.upgate3)
                .onEnter(()->{
                    PathChain up = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), gateintake3))
                            .setLinearHeadingInterpolation(follower.heading(), gateintake3.heading())
                            .setBrakingStrength(0.4)
                            .build();
                    follower.followPath(up, true);
                })
                .transitionTimed(0.4)

                 */

                .state(AutoStates.MOVETOSHOOT7)
                .onEnter(() -> {
                    shooter.setTurretPos(shooter.convertDegreestoServoPos(-77));
                    shooter.setHood(0.69);
                    shooter.setTargetVelocity(1370);
                    PathChain toScoreGate = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), shootPose2nd))
                            .setReversed()
                            .setBrakingStrength(0.6)
                            .build();
                    follower.followPath(toScoreGate, false);
                })

                .transition(() -> follower.atParametricEnd())
                .transition(() -> follower.getCurrentTValue() > 0.82).transitionTimed(1.5)
                .state(AutoStates.wait7)
                .onEnter(() -> {
                    follower.pausePathFollowing();
                    intakes.setPower(1);
                    shooter.setUpperGate(true);
                })
                .transitionTimed(0.1)
                .state(AutoStates.SHOOT7)
                .onEnter(() -> {
                    spindexer.setKicker(true);
                })


                .transitionTimed(0.3)
                .state(AutoStates.MOVETOINTAKE7)
                .onEnter(() -> {
                    spindexer.setKicker(false);
                    PathChain toIntakeGate = follower.pathBuilder()
                            .addPath(new BezierCurve(follower.getPose(), gateintakecontrol, gateintake4))
                            .setLinearHeadingInterpolation(follower.heading(), gateintake4.heading())
                            .setBrakingStrength(0.6)
                            .build();
                    follower.followPath(toIntakeGate, true);
                })
                .transitionTimed(0.15)
                .state(AutoStates.closegate7)
                .onEnter(() -> {
                    shooter.setUpperGate(false);
                })
                .transition(() -> intakes.getBeamBreakOutside() && intakes.getBeamBreakInside() && intakes.getDetected())
                .transitionTimed(2.35)
                /*
                .state(AutoStates.backupgate3)
                .onEnter(()->{
                    PathChain back = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), gateintakeback3))
                            .setLinearHeadingInterpolation(follower.heading(), gateintakeback3.heading())
                            .setBrakingStrength(0.4)
                            .build();
                    follower.followPath(back, true);
                })
                                .transitionTimed(0.4)
                .state(AutoStates.upgate3)
                .onEnter(()->{
                    PathChain up = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), gateintake3))
                            .setLinearHeadingInterpolation(follower.heading(), gateintake3.heading())
                            .setBrakingStrength(0.4)
                            .build();
                    follower.followPath(up, true);
                })
                .transitionTimed(0.4)

                 */

                .state(AutoStates.MOVETOSHOOT8)
                .onEnter(() -> {
                    shooter.setTurretPos(shooter.convertDegreestoServoPos(-79));
                    shooter.setHood(0.67);
                    shooter.setTargetVelocity(1370);
                    PathChain toScoreGate = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), shootPose2nd))
                            .setReversed()
                            .setBrakingStrength(0.6)
                            .build();
                    follower.followPath(toScoreGate, false);
                })

                .transition(() -> follower.atParametricEnd())
                .transition(() -> follower.getCurrentTValue() > 0.82)
                .transitionTimed(1.5)
                .state(AutoStates.wait8)
                .onEnter(() -> {
                    follower.pausePathFollowing();
                    intakes.setPower(1);
                    shooter.setUpperGate(true);
                })
                .transitionTimed(0.1)
                .state(AutoStates.SHOOT8)
                .onEnter(() -> {
                    spindexer.setKicker(true);
                })

                .build();

        autoMachine.start();

        while (opModeIsActive()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();
            Robot.savedPose = follower.getPose();
            autoMachine.update();
            follower.update();
            intakes.update();
            shooter.update();
            spindexer.update();
            telemetry.addData("State auto: ", autoMachine.getState());
            telemetry.addData("Shooter Target", shooter.getTargetVelo());
            telemetry.addData("Shooter Velocity", shooter.getCurrentVelocity());
            telemetry.addData("Spindexer kick", spindexer.kicked);
            telemetry.addData("Pose: ", follower.getPose());
            telemetry.update();
        }
    }
}


