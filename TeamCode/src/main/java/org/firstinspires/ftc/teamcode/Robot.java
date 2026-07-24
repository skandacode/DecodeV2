package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Light;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.subsystems.vision.LimelightCamera;
import org.firstinspires.ftc.teamcode.utils.Alliance;

import java.util.List;

import static org.firstinspires.ftc.teamcode.pedro.Constants.create;
//import static org.firstinspires.ftc.teamcode.pedro.Constants.createFollower;

public class Robot {
    private List<LynxModule> hubs;
    public Intake intake;
    public Kicker kicker;
    public Shooter shooter;
    public Follower follower;
    public Light light;
    public LimelightCamera limelight = null;
    public static Pose savedPose = Pose.zero();
    public Alliance alliance;
    private final Timer loop = new Timer();
    public double loops = 0, lastLoop = 0, loopTime = 0;

    public Robot(HardwareMap hardwareMap, Alliance alliance, boolean activateLimelight) {
        this.alliance = alliance;

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs)
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        kicker = new Kicker(hardwareMap);
        follower = create(hardwareMap);
        light = new Light(hardwareMap);

        loop.reset();

        if (activateLimelight)
            limelight = new LimelightCamera(hardwareMap);
    }

    public Robot(HardwareMap hardwareMap, Alliance alliance) {
        this(hardwareMap, alliance, false);
    }

    public void update() {
        loops++;

        if (loops > 10) {
            double now = loop.milliseconds();
            loopTime = (now - lastLoop) / loops;
            lastLoop = now;
            loops = 0;
        }

        clearCache();
        intake.update();
        kicker.update();
        shooter.update();
        light.update();
        follower.update();
        savedPose = follower.pose();
    }

    public void clearCache() {
        for (LynxModule hub : hubs) hub.clearBulkCache();
    }

    public void saveEnd() {
        savedPose = follower.pose();
    }

    public double getLoopTimeMs() {
        return loopTime;
    }

    public double getLoopTimeHz() {
        return 1000 / loopTime;
    }
}
