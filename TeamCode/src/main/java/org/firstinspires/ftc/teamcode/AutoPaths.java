//package org.firstinspires.ftc.teamcode;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.ivy.CommandBuilder;
//
//import org.firstinspires.ftc.teamcode.utils.Alliance;
//import org.firstinspires.ftc.teamcode.utils.FollowPath;
//
//import static com.pedropathing.api.Paths.curve;
//import static com.pedropathing.api.Paths.line;
//
//public class AutoPaths {
//    private final Follower f;
//    public Pose gateOpen, gatePreOpen, gatePreOpen2ndTime;
//    public Pose gateIntake1, gateIntakeBack1, gateIntake2, gateIntakeBack2, gateIntake3, gateIntakeBack3, gateIntake4, gateIntake5, gateIntakeControl;
//    public Pose start, startOld;
//    public Pose preloadScore, score, scoreGoingToLast, scoreLeave;
//    public Pose intake1Control, intake2Control, intake3Control, intake3Back, intake1End, intake2End, intake3End;
//    public Pose leave;
//
//    public AutoPaths(Follower follower, Alliance alliance) {
//        this.f = follower;
//
//        PoseFactory p;
//        if (alliance.equals(Alliance.RED))
//            p = PoseFactory.degrees().mirrorX(70.25);
//        else
//            p = PoseFactory.degrees();
//
//        gateOpen = p.of(7.75, 73.75, 270);
//        gatePreOpen = p.of(40.75, 73.75, 270);
//        gatePreOpen2ndTime = p.of(40.75, 66.75, 270);
//
//        gateIntake1 = p.of(7.75, 60.75, 250);
//        gateIntakeBack1 = p.of(10.75, 56.75, 250);
//        gateIntake2 = p.of(7.75, 61.25, 250);
//        gateIntakeBack2 = p.of(10.75, 56.75, 260);
//        gateIntake3 = p.of(7.75, 61.75, 250);
//        gateIntakeBack3 = p.of(10.75, 56.75, 250);
//        gateIntake4 = p.of(7.75, 62.05, 250);
//        gateIntake5 = p.of(7.75, 62.05, 250);
//
//        gateIntakeControl = p.of(28.75, 62.75, 347);
//
//        start = p.of(15.75, 114.75, 276);
//        startOld = p.of(15.25, 107.75, 270);
//
//        preloadScore = p.of(54.75, 102.75, 230);
//        score = p.of(53.75, 87.75, 220);
//        scoreGoingToLast = p.of(50.75, 90.75, 247);
//        scoreLeave = p.of(54.75, 100.75, 247);
//
//        intake1Control = p.of(43.75, 87.75, 270);
//        intake2Control = p.of(41.75, 57.75, 190);
//        intake3Control = p.of(50.75, 34.75, 78);
//        intake3Back = p.of(30.75, 36.75, 180);
//
//        intake1End = p.of(18.75, 86.75, 180);
//        intake2End = p.of(7.75, 60.75, 180);
//        intake3End = p.of(7.75, 36.75, 180);
//        leave = p.of(15.75, 87.75, 180);
//    }
//
//    public CommandBuilder preload() {
////        PathChain path = f.pathBuilder()
////                .addPath(new BezierLine(start, preloadScore))
////                .setLinearHeadingInterpolation(start.getHeading(), preloadScore.getHeading())
////                .setBrakingStrength(1.5)
////                .build();
////        return new FollowPath(this.f, path);
////
//        return new FollowPath(this.f, line(start, preloadScore).linear(start, preloadScore));
//    }
//
//    public CommandBuilder intakeSpike1() {
////        PathChain path = f.pathBuilder()
////                .addPath(new BezierCurve(preloadScore, intake1Control, intake1End))
////                .setTangentHeadingInterpolation()
////                .setNoDeceleration()
////                .build();
////        return new FollowPath(this.f, path);
//        return new FollowPath(this.f, curve(preloadScore, intake1Control, intake1End).tangent());
//    }
//
//    public CommandBuilder scoreSpike1() {
//        return new FollowPath(this.f, curve(intake1End, intake1Control, score).linear(intake1End, score));
//        // braking(1.5)
//    }
//
//    public CommandBuilder intakeSpike2() {
//        return new FollowPath(this.f, curve(score, intake2Control, intake2End).tangent());
//        // noDeceleration()
//    }
//
//    public CommandBuilder scoreSpike2() {
//        return new FollowPath(this.f, curve(intake2End, intake2Control, score).linear(intake2End, score));
//        // braking(1.5)
//    }
//
//    public CommandBuilder openGate2() {
//        return new FollowPath(this.f, line(gatePreOpen2ndTime, gateOpen).linear(gatePreOpen2ndTime, gateOpen));
//    }
//
//    public CommandBuilder openGate() {
//        return new FollowPath(this.f, line(gatePreOpen, gateOpen).linear(gatePreOpen, gateOpen));
//    }
//
//    public CommandBuilder backToGate1() {
//        return new FollowPath(this.f, line(intake1End, gatePreOpen).linear(intake1End, gatePreOpen));
//        // noDeceleration()
//    }
//
//    public CommandBuilder backToGate2() {
//        return new FollowPath(this.f, curve(intake2End, intake2Control, gatePreOpen2ndTime).linear(intake2End, gatePreOpen2ndTime));
//        // noDeceleration()
//    }
//
//    public CommandBuilder scoreGate1() {
//        return new FollowPath(this.f, line(gateOpen, score).linear(gateOpen, score));
//        // braking(1)
//    }
//
//    public CommandBuilder scoreGate2() {
//        return new FollowPath(this.f, line(gateOpen, score).reverseTangent());
//        // braking(1)
//    }
//
//    public CommandBuilder scoreGate() {
//        return new FollowPath(this.f, line(f.pose(), score));
//        // braking(0.6)
//    }
//
//    private CommandBuilder intakeGate(Pose target, double brakingStrength) {
//        return new FollowPath(this.f, curve(f.pose(), gateIntakeControl, target).linear(f.pose(), target));
//        // braking(brakingStrength)
//    }
//}
