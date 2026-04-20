package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

/**
 * Test OpMode: prints largest green/purple clump location to telemetry.
 */

@TeleOp
public class ArtifactPipelineTest extends LinearOpMode {

    private OpenCvWebcam webcam;
    private ArtifactPipeline pipeline;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new ArtifactPipeline();
        webcam.setPipeline(pipeline);
        webcam.setMillisecondsPermissionTimeout(5000);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera error", errorCode);
                telemetry.update();
            }
        });

        while (opModeInInit()) {
            telemetry.addData("Found", pipeline.isFound());
            telemetry.addData("Largest X", pipeline.getCenterX());
            telemetry.addData("Largest Y", pipeline.getCenterY());
            telemetry.addData("Area", "%.1f", pipeline.getBestArea());
            telemetry.addData("FPS", "%.2f", webcam.getFps());
            telemetry.update();
            sleep(75);
        }

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Found", pipeline.isFound());
            telemetry.addData("Largest X", pipeline.getCenterX());
            telemetry.addData("Largest Y", pipeline.getCenterY());
            telemetry.addData("Area", "%.1f", pipeline.getBestArea());
            telemetry.addData("Pipeline ms", webcam.getPipelineTimeMs());
            telemetry.update();
            sleep(50);
        }

        webcam.closeCameraDeviceAsync(() -> { });
    }
}
