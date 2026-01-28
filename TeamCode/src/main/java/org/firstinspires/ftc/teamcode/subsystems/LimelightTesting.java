package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class LimelightTesting extends LinearOpMode {
    LimelightCamera limelightCamera;

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
        limelightCamera.setCurrentPipeline(LimelightCamera.Pipelines.BALLTRACKING);
        while (opModeIsActive()){
            LLFieldScannerResults results = limelightCamera.getTrackingResults();

            if (results == null){
                telemetry.addLine("is null");
            }else {
                telemetry.addLine(results.toString());
                telemetry.addData("Position", results.getPosition());
            }
            telemetry.update();
        }
    }
}
