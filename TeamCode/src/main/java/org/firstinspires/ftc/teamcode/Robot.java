package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.pedro.PanelsDrawing;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Light;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.vision.LimelightCamera;
import org.firstinspires.ftc.teamcode.utils.Alliance;

import java.util.List;

import static org.firstinspires.ftc.teamcode.pedro.Constants.createFollower;

public class Robot {
    private List<LynxModule> hubs;
    public Intake intake;
    public Transfer transfer;
    public Shooter shooter;
    public Follower follower;
    public Light light;
    public LimelightCamera limelight = null;
    public static Pose endPose = new Pose();
    public Alliance alliance;


    public Robot(HardwareMap hardwareMap, Alliance alliance, boolean activateLimelight) {
        this.alliance = alliance;

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs)
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        transfer = new Transfer(hardwareMap);
        follower = createFollower(hardwareMap);
        light = new Light(hardwareMap);

        if (activateLimelight)
            limelight = new LimelightCamera(hardwareMap);
    }

    public Robot(HardwareMap hardwareMap, Alliance alliance) {
        this(hardwareMap, alliance, false);
    }

    public void update() {
        clearCache();
        intake.update();
        transfer.update();
        shooter.update();
        light.update();
        follower.update();
    }

    public void clearCache() {
        for (LynxModule hub : hubs) hub.clearBulkCache();
    }

    public void saveEnd() {
        endPose = follower.getPose();
    }
}
