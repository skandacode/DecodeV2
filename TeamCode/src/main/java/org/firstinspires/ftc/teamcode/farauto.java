package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.*;
import static org.firstinspires.ftc.teamcode.pedro.Constants.create;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
//import org.firstinspires.ftc.teamcode.subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;
//import org.firstinspires.ftc.teamcode.subsystems.LimelightCamera;
//import org.firstinspires.ftc.teamcode.subsystems.Position;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.Arrays;
import java.util.List;

@Configurable
@Autonomous(name = "farauto", group = "Auto")
public class farauto extends LinearOpMode {
    private Follower follower;
    Intake intakes;
    String colorAlliance = "REDE";
    int Posmultiplier = -1;
    Shooter shooter;
    Kicker spindexer;
    public boolean shooterButton = false;
    public double shootWaitTime = 0.25;
    public double intakerejectspeed = 1;
    public double limelightAdjust=0.0;

    Limelight3A limelight;

    Path path;
    List<LynxModule> hubs;


    public static Shooter.Goal shooterTarget = Shooter.Goal.RED;


    public enum AutoStates {
        MOVETOSHOOT1, wait1, preSHOOT1,SHOOT1,
        MOVETOINTAKE1,
        MOVETOSHOOT2, wait2, preSHOOT2,SHOOT2,
        MOVETOINTAKE2,
        MOVETOSHOOT3, reject3, wait3,preSHOOT3, SHOOT3,
        MOVETOINTAKE3,
        MOVETOSHOOT4, reject4, wait4,preSHOOT4, SHOOT4,
        MOVETOINTAKE4,
        MOVETOSHOOT5, reject5, wait5,preSHOOT5, SHOOT5,
        MOVETOINTAKE5,
        MOVETOSHOOT6,reject6, wait6,preSHOOT6, SHOOT6,
        MOVETOINTAKE6,
        MOVETOSHOOT7, reject7, wait7,preSHOOT7, SHOOT7,
        MOVETOINTAKE7,
        MOVETOSHOOT8, reject8,wait8,preSHOOT8, SHOOT8,
        MOVETOINTAKE8,
        MOVETOSHOOT9, reject9,wait9,preSHOOT9, SHOOT9,
        park
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs)
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        limelight.pipelineSwitch(0);
        limelight.start();
        intakes = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        spindexer = new Kicker(hardwareMap);
        follower = create(hardwareMap);
//        follower = create(hardwareMap);

        Pose startPose = new Pose(57, -13.5 * Posmultiplier, Math.toRadians(-90 * Posmultiplier));
        follower.setPose(startPose);

        while (opModeInInit()) {
//            for (LynxModule hub : hubs) hub.clearBulkCache();
            follower.setPose(startPose);
            follower.update();
            telemetry.addData("Init Pose: ", follower.pose());
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
            spindexer.setKicker(false);
            Shooter.turretOffset=-3.5;
            shooter.setTurretPos(shooter.convertDegreestoServoPos(0));
            shooter.setHood(0.65);
            shooter.setUpperGate(false);
            shooter.update();
            telemetry.update();
            spindexer.update();
        }

        if (opModeIsActive()) {
            waitForStart();
            Pose shootPose1 = new Pose(57, -13.5 * Posmultiplier, Math.toRadians(-90 * Posmultiplier));

            Pose shootPose = new Pose(50, -20 * Posmultiplier, Math.toRadians(-90 * Posmultiplier));
            Pose intakeHuman = new Pose(58, -61 * Posmultiplier, Math.toRadians(-90 * Posmultiplier));
            Pose intake1Pose = new Pose(20, -28 * Posmultiplier, Math.toRadians(-90 * Posmultiplier));
            Pose intake1donePose = new Pose(33, -61 * Posmultiplier, Math.toRadians(-90 * Posmultiplier));


            Pose intakemidPose = new Pose(44, -17 * Posmultiplier, Math.toRadians(-90 * Posmultiplier));
            Pose intakemiddonePose = new Pose(44, -63 * Posmultiplier, Math.toRadians(-90 * Posmultiplier));
            Pose leave = new Pose(33, -30 * Posmultiplier, Math.toRadians(-90 * Posmultiplier));


            follower.setPose(startPose);
            follower.update();

            StateMachine autoMachine = new StateMachineBuilder() //Autonomia
                    .state(AutoStates.MOVETOSHOOT1)
                    .onEnter(() -> {
                        intakes.setPower(0.4);
                        spindexer.setKicker(false);
                        //shooter.setHood(0.67);
                        //shooter.setTargetVelocity(1900);
//                        shooter.setTurretPos(shooter.convertDegreestoServoPos(72*Posmultiplier));
                    })
                    .transitionTimed(1.1)

                    .state(AutoStates.preSHOOT1)
                    .onEnter(() -> {
                        shooter.setUpperGate(true);
                    })
                    .transitionTimed(0.1)
                    .state(AutoStates.SHOOT1)
                    .onEnter(() -> {
                        intakes.setIntakePower(1);
                        intakes.setTransferPower(0.7);
                        spindexer.setKicker(true);
                    })
                    .transitionTimed(0.26)


                    .state(AutoStates.MOVETOINTAKE1)
                    .onEnter(() -> {
                        intakes.setPower(1);
                        spindexer.setKicker(false);
                        shooter.setUpperGate(false);
//                        shooter.setTurretPos(shooter.convertDegreestoServoPos(71 *Posmultiplier));
//                        shooter.setHood(0.67);
//                        shooter.setTargetVelocity(1900);

                        Path toIntake = curve(follower.pose(), intake1Pose, intake1donePose).constant(intake1donePose);
                        follower.follow(toIntake);
                    })
                    .transitionTimed(1.5)

                    .state(AutoStates.MOVETOSHOOT2)
                    .onEnter(() -> {
                        intakes.setTransferPower(0.5);
                        Path toScore = line(follower.pose(), shootPose).constant(shootPose);
                        follower.follow(toScore);
                    })
                    .transition(() -> follower.atParametricEnd())
                    .transitionTimed(2.4)

                    .state(AutoStates.wait2)
                    .onEnter(() -> {
                    })
                    .transitionTimed(0.2)

                    .state(AutoStates.preSHOOT2)
                    .onEnter(() -> {
                        shooter.setUpperGate(true);
                    })
                    .transitionTimed(0.1)
                    .state(AutoStates.SHOOT2)
                    .onEnter(() -> {
                        intakes.setIntakePower(1);
                        intakes.setTransferPower(0.7);
                        spindexer.setKicker(true);
                    })
                    .transitionTimed(0.3)
                    .state(AutoStates.MOVETOINTAKE2)
                    .onEnter(() -> {
                        intakes.setPower(1);
                        shooter.setUpperGate(false);
                        spindexer.setKicker(false);

//                        shooter.setTurretPos(shooter.convertDegreestoServoPos(72*Posmultiplier));

                        Path toIntakeHuman = line(follower.pose(), intakeHuman).constant(intakeHuman);
                        follower.follow(toIntakeHuman);
                    })
                    .transitionTimed(1.7)

                    .state(AutoStates.MOVETOSHOOT3)
                    .onEnter(() -> {
//                        intakes.setTransferPower(0.4);
                        Path toScore = line(follower.pose(), shootPose).constant(shootPose);
                        follower.follow(toScore);
                    })
                    .transition(() -> follower.closestT() > 0.8)
                    .transitionTimed(1.9)
                    .state(AutoStates.wait3)
                    .onEnter(() -> {
                        follower.hold(shootPose);
                    })
                    .transitionTimed(0.36)

                    .state(AutoStates.preSHOOT3)
                    .onEnter(() -> {
                        shooter.setUpperGate(true);
                    })
                    .transitionTimed(0.1)
                    .state(AutoStates.SHOOT3)
                    .onEnter(() -> {
                        intakes.setIntakePower(1);
                        intakes.setTransferPower(0.7);
                        spindexer.setKicker(true);
                    })
                    .transitionTimed(0.3)
                    .state(AutoStates.MOVETOINTAKE3)
                    .onEnter(() -> {
                        intakes.setPower(1);
//                        shooter.setTurretPos(shooter.convertDegreestoServoPos(72*Posmultiplier));
                        shooter.setUpperGate(false);
                        spindexer.setKicker(false);

                        detection();
                        follower.follow(path);
                    })
                    .transitionTimed(1.2)
                    .state(AutoStates.MOVETOSHOOT4)
                    .onEnter(() -> {
//                        intakes.setPower(0.4);
                        Path toScore = line(follower.pose(), shootPose).constant(shootPose);
                        follower.follow(toScore);
                    })
                    .transition(() -> follower.closestT() > 0.8)
                    .transitionTimed(1.9)

                    .state(AutoStates.wait4)
                    .onEnter(() -> {
follower.hold(shootPose);
                    })
                    .transitionTimed(0.36)

                    .state(AutoStates.preSHOOT4)
                    .onEnter(() -> {
                        shooter.setUpperGate(true);
                    })
                    .transitionTimed(0.1)
                    .state(AutoStates.SHOOT4)
                    .onEnter(() -> {
                        intakes.setIntakePower(1);
                        intakes.setTransferPower(0.7);
                        spindexer.setKicker(true);
                    })
                    .transitionTimed(0.3)
                    .state(AutoStates.MOVETOINTAKE4)
                    .onEnter(() -> {
                        intakes.setPower(1);
                        shooter.setUpperGate(false);
                        spindexer.setKicker(false);

                        detection();
                        follower.follow(path);
                    })
                    .transitionTimed(1.5)
                    .state(AutoStates.MOVETOSHOOT5)
                    .onEnter(() -> {
//                        intakes.setPower(0.4);
                        Path toScore = line(follower.pose(), shootPose).constant(intake1donePose);
                        follower.follow(toScore);
                    })
                    .transition(() -> follower.closestT() > 0.8)
                    .transitionTimed(1.9)
                    .state(AutoStates.wait5)
                    .onEnter(() -> {
follower.hold(shootPose);                    })
                    .transitionTimed(0.36)

                    .state(AutoStates.preSHOOT5)
                    .onEnter(() -> {
                        shooter.setUpperGate(true);
                    })
                    .transitionTimed(0.1)
                    .state(AutoStates.SHOOT5)
                    .onEnter(() -> {
                        intakes.setIntakePower(1);
                        intakes.setTransferPower(0.7);
                        spindexer.setKicker(true);
                    })
                    .transitionTimed(0.3)
                    .state(AutoStates.MOVETOINTAKE5)
                    .onEnter(() -> {
                        intakes.setPower(1);
//                        shooter.setTurretPos(shooter.convertDegreestoServoPos(72*Posmultiplier));
                        shooter.setUpperGate(false);
                        spindexer.setKicker(false);
                        detection();
                        follower.follow(path);
                    })
                    .transitionTimed(1.5)

                    .state(AutoStates.MOVETOSHOOT6)
                    .onEnter(() -> {
//                        intakes.setPower(0.4);
                        Path toScore = line(follower.pose(), shootPose).constant(shootPose);
                        //shooter.setTurretPos(shooter.convertDegreestoServoPos(69*Posmultiplier));

                        follower.follow(toScore);
                    })
                    .transition(() -> follower.closestT() > 0.8)
                    .transitionTimed(1.9)

                    .state(AutoStates.wait6)
                    .onEnter(() -> {
follower.hold(shootPose);                    })
                    .transitionTimed(0.36)

                    .state(AutoStates.preSHOOT6)
                    .onEnter(() -> {
                        shooter.setUpperGate(true);
                    })
                    .transitionTimed(0.1)
                    .state(AutoStates.SHOOT6)
                    .onEnter(() -> {
                        intakes.setIntakePower(1);
                        intakes.setTransferPower(0.7);
                        spindexer.setKicker(true);
                    })
                    .transitionTimed(0.3)
                    .state(AutoStates.MOVETOINTAKE6)
                    .onEnter(() -> {
                        intakes.setPower(1);
                        shooter.setUpperGate(false);
                        spindexer.setKicker(false);
                        detection();
                        follower.follow(path);
                    })
                    .transitionTimed(1.6)
                    .state(AutoStates.MOVETOSHOOT7)
                    .onEnter(() -> {
//                        intakes.setPower(0.4);
                        Path toScore = line(follower.pose(), shootPose).constant(shootPose);
//                        shooter.setTurretPos(shooter.convertDegreestoServoPos(72*Posmultiplier));

                        follower.follow(toScore);
                    })
                    .transition(() -> follower.closestT() > 0.8)
                    .transitionTimed(1.9)
                    .state(AutoStates.wait7)
                    .onEnter(() -> {
follower.hold(shootPose);                    })
                    .transitionTimed(0.36)

                    .state(AutoStates.preSHOOT7)
                    .onEnter(() -> {
                        shooter.setUpperGate(true);
                    })
                    .transitionTimed(0.1)
                    .state(AutoStates.SHOOT7)
                    .onEnter(() -> {
                        intakes.setIntakePower(1);
                        intakes.setTransferPower(0.7);
                        spindexer.setKicker(true);
                    })
                    .transitionTimed(0.3)
                    .state(AutoStates.MOVETOINTAKE7)
                    .onEnter(() -> {
//                        intakes.setPower(1);
                        shooter.setUpperGate(false);
                        spindexer.setKicker(false);
                        detection();
                        follower.follow(path);
                    })
                    .transitionTimed(1.6)
                    .state(AutoStates.MOVETOSHOOT8)
                    .onEnter(() -> {
//                        intakes.setPower(0.4);
                        Path toScore = line(follower.pose(), shootPose).constant(shootPose);
//                        shooter.setTurretPos(shooter.convertDegreestoServoPos(72*Posmultiplier));

                        follower.follow(toScore);
                    })
                    .transition(() -> follower.closestT() > 0.8)
                    .transitionTimed(1.9)

                    .state(AutoStates.wait8)
                    .onEnter(() -> {
follower.hold(shootPose);                    })
                    .transitionTimed(0.36)

                    .state(AutoStates.preSHOOT8)
                    .onEnter(() -> {
                        shooter.setUpperGate(true);
                    })
                    .transitionTimed(0.1)
                    .state(AutoStates.SHOOT8)
                    .onEnter(() -> {
                        intakes.setIntakePower(1);
                        intakes.setTransferPower(0.7);
                        spindexer.setKicker(true);
                    })
                    .transitionTimed(0.3)
                    .state(AutoStates.MOVETOINTAKE8)
                    .onEnter(() -> {
                        intakes.setPower(1);
                        shooter.setUpperGate(false);
                        spindexer.setKicker(false);
                        detection();
                        follower.follow(path);

                    })
                    .transitionTimed(1.6)
                    .state(AutoStates.MOVETOSHOOT9)
                    .onEnter(() -> {
//                        intakes.setPower(0.4);
                        Path toScore = line(follower.pose(), shootPose).constant(shootPose);
//                        shooter.setTurretPos(shooter.convertDegreestoServoPos(72*Posmultiplier));

                        follower.follow(toScore);
                    })
                    .transition(() -> follower.closestT() > 0.8)
                    .transitionTimed(1.9)
                    .state(AutoStates.wait9)
                    .onEnter(() -> {
                        follower.hold(shootPose);
                    })
                    .transitionTimed(0.36)

                    .state(AutoStates.preSHOOT9)
                    .onEnter(() -> {
                        shooter.setUpperGate(true);
                    })
                    .transitionTimed(0.1)
                    .state(AutoStates.SHOOT9)
                    .onEnter(() -> {
                        intakes.setIntakePower(1);
                        intakes.setTransferPower(0.7);
                        spindexer.setKicker(true);
                    })
                    .transitionTimed(0.3)
                    .state(AutoStates.park)
                    .onEnter(() -> {
                        shooter.setUpperGate(false);
                        spindexer.setKicker(false);
                        Path park = line(follower.pose(), leave).constant(leave);
                        follower.follow(park);
                    })
                    .build();

            autoMachine.start();
            //limelightCamera.setCurrentPipeline(LimelightCamera.Pipelines.BALLTRACKING);
            while (opModeIsActive()) {
                for (LynxModule hub : hubs) hub.clearBulkCache();
                shooter.aimTurret(follower.pose(),shooterTarget);
                shooter.setHood(0.66);
                shooter.setTargetVelocity(1860);
                Robot.savedPose = follower.pose();
                telemetry.addData("Angle and distance:", Arrays.toString(shooter.getAngleDistance(Robot.savedPose, shooterTarget)));
                follower.update();
                intakes.update();
                shooter.update();
                autoMachine.update();
                spindexer.update();
                telemetry.addData("t-value", follower.closestT());
                telemetry.addData("heading +90: ", follower.pose().heading()*Posmultiplier);
                telemetry.addData("heading error: ", Math.toDegrees(follower.pose().heading() - Math.PI/2));
                telemetry.addData("State auto: ", autoMachine.getState());
                telemetry.addData("Shooter Target", shooter.getTargetVelo());
                telemetry.addData("Shooter Velocity", shooter.getCurrentVelocity());
                telemetry.addData("Spindexer kick", spindexer.kicked);
                telemetry.addData("Pose: ", follower.pose());
                telemetry.update();
            }
        }
        limelight.stop();
    }
    LLResult result;
    public static double timp = 1.0;
    double tx,ty,x, y, x2, y2;
    Pose ball1, ball2, ball3, p;
    public void detection(){
        p=follower.pose();
        System.out.println("detection p: "+p.toString());
        result = limelight.getLatestResult();
        tx = result.getTx();
        ty = result.getTy();

        if (tx==0 && ty==0){
            path = line(p, new Pose(56, -61 * Posmultiplier, Math.toRadians(-90 * Posmultiplier))).constant(90);
            return;
        }
        x = 5.25*Math.sin(Math.toRadians(tx))/Math.tan(Math.toRadians(ty));
        y = -5.25*Math.cos(Math.toRadians(tx))/Math.tan(Math.toRadians(ty));
        ball1 = p.plus(new Pose(-x, y));
        System.out.println("Detection angles: "+tx +" "+ty);
        System.out.println("Detection inches: "+x +" "+y);

        eepy(100);
        result = limelight.getLatestResult();
        tx = result.getTx();
        ty = result.getTy();

        if (tx==0 && ty==0){
            path = line(p, new Pose(56, -61 * Posmultiplier, Math.toRadians(-90 * Posmultiplier))).constant(90);
            return;
        }

        x = 5.25*Math.sin(Math.toRadians(tx))/Math.tan(Math.toRadians(ty));
        y = -5.25*Math.cos(Math.toRadians(tx))/Math.tan(Math.toRadians(ty));
        System.out.println("Detection angles: "+tx +" "+ty);
        System.out.println("Detection inches: "+x +" "+y);

        ball2 = p.plus(new Pose(-x, y));
        ball3 = ball1.plus(new Pose(10*timp*(ball2.x()-ball1.x()),10*timp*(ball2.y()-ball1.y())));
        ball3=ball3.withY(60).withHeading(Math.toRadians(90));
        if (ball3.x()>58){
            ball3 = ball3.withX(58);
        }
        if (ball3.x()<36){
            ball3 = ball3.withX(36);
        }

        path = line(p, ball3).constant(p.heading());

        System.out.println("detection ball 1"+ ball1.toString());
        System.out.println("detection ball 2"+ ball2.toString());
        System.out.println("detection ball 3"+ ball3.toString());

    }
    public void eepy(long ms){
        ElapsedTime timer = new ElapsedTime();
        while (timer.milliseconds()<ms){
            for (LynxModule hub : hubs) hub.clearBulkCache();
            Robot.savedPose = follower.pose();
            telemetry.addData("Angle and distance:", Arrays.toString(shooter.getAngleDistance(Robot.savedPose, shooterTarget)));
            follower.update();
            intakes.update();
            shooter.update();
            spindexer.update();
            telemetry.addData("t-value", follower.closestT());
            telemetry.addData("heading +90: ", follower.pose().heading()*Posmultiplier);
            telemetry.addData("heading error: ", Math.toDegrees(follower.pose().heading() - Math.PI/2));
            telemetry.addData("Shooter Target", shooter.getTargetVelo());
            telemetry.addData("Shooter Velocity", shooter.getCurrentVelocity());
            telemetry.addData("Spindexer kick", spindexer.kicked);
            telemetry.addData("Pose: ", follower.pose());
            telemetry.update();
        }
    }
}
