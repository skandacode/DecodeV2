package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Arrays;

@TeleOp
public class LimelightTesting extends LinearOpMode {
    LimelightCamera limelightCamera;

    LimelightCamera.Pipelines colorPipeline = LimelightCamera.Pipelines.BLUETRACK;

    @Override
    public void runOpMode() throws InterruptedException {
        limelightCamera = new LimelightCamera(hardwareMap);

        limelightCamera.setCurrentPipeline(LimelightCamera.Pipelines.MOTIF);

        while (opModeInInit()){
            int motif = limelightCamera.getMotif();

            telemetry.addData("Motif", motif);
            telemetry.update();
        }

        waitForStart();
        limelightCamera.setCurrentPipeline(colorPipeline);
        while (opModeIsActive()){
            telemetry.addData("results", limelightCamera.getTrackingResults());
            telemetry.update();
        }
    }
}
