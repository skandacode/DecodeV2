package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.pedro.Constants.createFollower;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.Arrays;

@TeleOp
@Configurable
public class ShooterTesting extends LinearOpMode {
    Shooter shooter;
    Intake intake;
    Follower follower;
    public static double targetVelocity = 0.0;
    public static double turretPos = 0.5;
    public static double hoodPos = 0.5;
    public static double intakePower = 0;
    public static Shooter.Goal targetGoal = Shooter.Goal.RED;

    public static double turretAngle = 0;
    public static boolean useTurretAngle = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);

        follower = createFollower(hardwareMap);
        follower.setPose(new Pose(60, 0, Math.toRadians(180)));

        waitForStart();

//        follower.startTeleopDrive();

        while (opModeIsActive()){
            intake.setPower(intakePower);
            shooter.setTargetVelocity(targetVelocity);
            shooter.setUpperGate(true);
            shooter.setHood(hoodPos);
            if (useTurretAngle){
                shooter.setTurretPos(shooter.convertDegreestoServoPos(turretAngle));
            }else {
                shooter.setTurretPos(turretPos);
            }
            telemetry.addData("angle distance", Arrays.toString(shooter.getAngleDistance(follower.pose(), targetGoal)));
            intake.update();
            shooter.update();
            follower.update();
            telemetry.addData("Shooter Velocity", shooter.getCurrentVelocity());
            telemetry.update();
        }
    }
}
