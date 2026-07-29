//package org.firstinspires.ftc.teamcode;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.ivy.CommandBuilder;
//import com.pedropathing.paths.PathChain;
//
//import org.firstinspires.ftc.teamcode.utils.Alliance;
//import org.firstinspires.ftc.teamcode.utils.FollowPath;
//
//public class Paths {
//    private final Follower f;
//
//    public Pose gateOpen = new Pose(7.75, 73.75, Math.toRadians(270));
//    public Pose gatePreOpen = new Pose(40.75, 73.75, Math.toRadians(270));
//    public Pose gatePreOpen2ndTime = new Pose(40.75, 66.75, Math.toRadians(270));
//
//    public Pose gateIntake1 = new Pose(7.75, 60.75, Math.toRadians(250));
//    public Pose gateIntakeBack1 = new Pose(10.75, 56.75, Math.toRadians(250));
//    public Pose gateIntake2 = new Pose(7.75, 61.25, Math.toRadians(250));
//    public Pose gateIntakeBack2 = new Pose(10.75, 56.75, Math.toRadians(260));
//    public Pose gateIntake3 = new Pose(7.75, 61.75, Math.toRadians(250));
//    public Pose gateIntakeBack3 = new Pose(10.75, 56.75, Math.toRadians(250));
//    public Pose gateIntake4 = new Pose(7.75, 62.05, Math.toRadians(250));
//    public Pose gateIntake5 = new Pose(7.75, 62.05, Math.toRadians(250));
//
//    public Pose gateIntakeControl = new Pose(28.75, 62.75, Math.toRadians(347));
//
//    public Pose start = new Pose(15.75, 114.75, Math.toRadians(276));
//    public Pose startOld = new Pose(15.25, 107.75, Math.toRadians(270));
//
//    public Pose preloadScore = new Pose(54.75, 102.75, Math.toRadians(230));
//    public Pose score = new Pose(53.75, 87.75, Math.toRadians(220));
//    public Pose scoreGoingToLast = new Pose(50.75, 90.75, Math.toRadians(247));
//    public Pose scoreLeave = new Pose(54.75, 100.75, Math.toRadians(247));
//
//    public Pose intake1Control = new Pose(43.75, 87.75, Math.toRadians(270));
//    public Pose intake2Control = new Pose(41.75, 57.75, Math.toRadians(190));
//    public Pose intake3Control = new Pose(50.75, 34.75, Math.toRadians(78));
//    public Pose intake3Back = new Pose(30.75, 36.75, Math.toRadians(180));
//
//    public Pose intake1End = new Pose(18.75, 86.75, Math.toRadians(180));
//    public Pose intake2End = new Pose(7.75, 60.75, Math.toRadians(180));
//    public Pose intake3End = new Pose(7.75, 36.75, Math.toRadians(180));
//    public Pose leave = new Pose(15.75, 87.75, Math.toRadians(180));
//
//    public Paths(Follower follower, Alliance alliance) {
//        this.f = follower;
//
//        if (alliance.equals(Alliance.RED)) {
//            gateOpen = gateOpen.mirror();
//            gatePreOpen = gatePreOpen.mirror();
//            gatePreOpen2ndTime = gatePreOpen2ndTime.mirror();
//
//            gateIntake1 = gateIntake1.mirror();
//            gateIntakeBack1 = gateIntakeBack1.mirror();
//            gateIntake2 = gateIntake2.mirror();
//            gateIntakeBack2 = gateIntakeBack2.mirror();
//            gateIntake3 = gateIntake3.mirror();
//            gateIntakeBack3 = gateIntakeBack3.mirror();
//            gateIntake4 = gateIntake4.mirror();
//
//            gateIntakeControl = gateIntakeControl.mirror();
//
//            start = start.mirror();
//            startOld = startOld.mirror();
//
//            preloadScore = preloadScore.mirror();
//            score = score.mirror();
//            scoreGoingToLast = scoreGoingToLast.mirror();
//            scoreLeave = scoreLeave.mirror();
//
//            intake1Control = intake1Control.mirror();
//            intake2Control = intake2Control.mirror();
//            intake3Control = intake3Control.mirror();
//            intake3Back = intake3Back.mirror();
//
//            intake1End = intake1End.mirror();
//            intake2End = intake2End.mirror();
//            intake3End = intake3End.mirror();
//            leave = leave.mirror();
//        }
//    }
//
//    public CommandBuilder preload() {
//        PathChain path = f.pathBuilder()
//                .addPath(new BezierLine(start, preloadScore))
//                .setLinearHeadingInterpolation(start.heading(), preloadScore.heading())
//                .setBrakingStrength(1.5)
//                .build();
//        return new FollowPath(this.f, path);
//    }
//
//    public CommandBuilder intakeSpike1() {
//        PathChain path = f.pathBuilder()
//                .addPath(new BezierCurve(preloadScore, intake1Control, intake1End))
//                .setTangentHeadingInterpolation()
//                .setNoDeceleration()
//                .build();
//        return new FollowPath(this.f, path);
//    }
//
//    public CommandBuilder scoreSpike1() {
//        PathChain path = f.pathBuilder()
//                .addPath(new BezierCurve(intake1End, intake1Control, score))
//                .setLinearHeadingInterpolation(intake1End.heading(), score.heading())
//                .setBrakingStrength(1.5)
//                .build();
//        return new FollowPath(this.f, path);
//    }
//
//    public CommandBuilder intakeSpike2() {
//        PathChain path = f.pathBuilder()
//                .addPath(new BezierCurve(score, intake2Control, intake2End))
//                .setTangentHeadingInterpolation()
//                .setNoDeceleration()
//                .build();
//        return new FollowPath(this.f, path);
//    }
//
//    public CommandBuilder scoreSpike2() {
//        PathChain path = f.pathBuilder()
//                .addPath(new BezierCurve(intake2End, intake2Control, score))
//                .setLinearHeadingInterpolation(intake2End.heading(), score.heading())
//                .setBrakingStrength(1.5)
//                .build();
//        return new FollowPath(this.f, path);
//    }
//
//    public CommandBuilder openGate2() {
//        PathChain path = f.pathBuilder()
//                .addPath(new BezierLine(gatePreOpen2ndTime, gateOpen))
//                .setLinearHeadingInterpolation(gatePreOpen2ndTime.heading(), gateOpen.heading())
//                .build();
//        return new FollowPath(this.f, path);
//    }
//
//    public CommandBuilder openGate() {
//        PathChain path = f.pathBuilder()
//                .addPath(new BezierLine(gatePreOpen, gateOpen))
//                .setLinearHeadingInterpolation(gatePreOpen.heading(), gateOpen.heading())
//                .build();
//        return new FollowPath(this.f, path);
//    }
//
//    public CommandBuilder backToGate1() {
//        PathChain path = f.pathBuilder()
//                .addPath(new BezierLine(intake1End, gatePreOpen))
//                .setNoDeceleration()
//                .setLinearHeadingInterpolation(intake1End.heading(), gatePreOpen.heading())
//                .build();
//        return new FollowPath(this.f, path);
//    }
//
//    public CommandBuilder backToGate2() {
//        PathChain path = f.pathBuilder()
//                .addPath(new BezierCurve(intake2End, intake2Control, gatePreOpen2ndTime))
//                .setNoDeceleration()
//                .setLinearHeadingInterpolation(intake2End.heading(), gatePreOpen2ndTime.heading())
//                .build();
//        return new FollowPath(this.f, path);
//    }
//
//    public CommandBuilder scoreGate1() {
//        PathChain path = f.pathBuilder()
//                .addPath(new BezierLine(gateOpen, score))
//                .setLinearHeadingInterpolation(gateOpen.heading(), score.heading())
//                .setBrakingStrength(1)
//                .build();
//        return new FollowPath(this.f, path);
//    }
//
//    public CommandBuilder scoreGate2() {
//        PathChain path = f.pathBuilder()
//                .addPath(new BezierLine(gateOpen, score))
//                .setBrakingStrength(1)
//                .setTangentHeadingInterpolation()
//                .setReversed()
//                .build();
//        return new FollowPath(this.f, path);
//    }
//
//    public CommandBuilder intakeGate1(double brakingStrength) {
//        return intakeGate(gateIntake1, brakingStrength);
//    }
//
//    public CommandBuilder intakeGate2(double brakingStrength) {
//        return intakeGate(gateIntake2, brakingStrength);
//    }
//
//    public CommandBuilder intakeGate3(double brakingStrength) {
//        return intakeGate(gateIntake3, brakingStrength);
//    }
//
//    public CommandBuilder intakeGate4(double brakingStrength) {
//        return intakeGate(gateIntake4, brakingStrength);
//    }
//
//    public CommandBuilder intakeGate5(double brakingStrength) {
//        return intakeGate(gateIntake5, brakingStrength);
//    }
//
//    public CommandBuilder scoreGate() {
//        PathChain path = f.pathBuilder()
//                .addPath(new BezierLine(f.getPose(), score))
//                .setReversed()
//                .setBrakingStrength(0.6)
//                .build();
//        return new FollowPath(this.f, path);
//    }
//
//    private CommandBuilder intakeGate(Pose target, double brakingStrength) {
//        PathChain path = f.pathBuilder()
//                .addPath(new BezierCurve(f.getPose(), gateIntakeControl, target))
//                .setLinearHeadingInterpolation(f.heading(), target.heading())
//                .setBrakingStrength(brakingStrength)
//                .build();
//        return new FollowPath(this.f, path);
//    }
//}
