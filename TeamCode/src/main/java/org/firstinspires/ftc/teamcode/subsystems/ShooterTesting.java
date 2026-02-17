package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Arrays;

@TeleOp
@Configurable
public class ShooterTesting extends LinearOpMode {
    Shooter shooter;
    Intakes intakes;
    Follower follower;
    public static double targetVelocity = 0.0;
    public static double turretPos = 0.5;
    public static double hoodPos = 0.5;
    public static double intakePower = 0;
    public static Shooter.Goal targetGoal = Shooter.Goal.RED;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        shooter = new Shooter(hardwareMap);
        intakes = new Intakes(hardwareMap);

        follower = createFollower(hardwareMap);
        follower.setStartingPose(new Pose(65, 0, Math.toRadians(180)));

        waitForStart();

        follower.startTeleopDrive();

        while (opModeIsActive()){
            intakes.setGoodIntakePower(intakePower);
            shooter.setTargetVelocity(targetVelocity);
            shooter.setUpperGateOpen(true);
            shooter.setHood(hoodPos);
            shooter.setTurretPos(turretPos);
            telemetry.addData("angle distance", Arrays.toString(shooter.getAngleDistance(follower.getPose(), targetGoal)));
            intakes.update();
            shooter.update();
            follower.update();
            telemetry.addData("Shooter Velocity", shooter.getCurrentVelocity());
            telemetry.update();
        }
    }
}
