# FTC Camera Code — Full Context and Sample OpMode

This file contains all camera-related code extracted and annotated from the team codebase, with all bugs fixed, plus a complete standalone sample opmode demonstrating the full camera pipeline pattern for red/blue target tracking.

### Bugs Fixed in This Version

| Bug | Location | Fix Applied |
|-----|----------|-------------|
| `midValue` divides by `RIGHT_ROI.area()` instead of `MID_ROI.area()` | `BluePipeline`, `RedPipeline` | Changed divisor to `MID_ROI.area()` |
| Red hue wraparound — only lower range (0–13) captured, misses 165–179 | `RedPipeline` | Added second `inRange` (170–179) OR'd with the first |
| Dead fields `upperthresh` / `lowerthresh` declared but never written to | `RedPipeline` | Removed; replaced with `redLow` / `redHigh` which are actually used |
| Missing tie-breaking logic | `RedPipeline` | Added same tie-breaking block as `BluePipeline` |
| `telemetry.addData` called inside `processFrame` | `RedPipeline` | Removed — telemetry belongs in the opmode, not the pipeline |
| `volatile` missing on result field | `BluePipeline`, `RedPipeline` | Added — required for safe cross-thread reads |

---

## Part 1: Pipeline Classes (Fixed Production Code)

### BluePipeline.java — Fixed and Annotated

```java
package org.firstinspires.ftc.teamcode.auto.notshown;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class BluePipeline extends OpenCvPipeline {

    Telemetry telemetry;

    // -------------------------------------------------------------------------
    // ROI definitions — coordinates in pixels for a 320x240 frame.
    // Each Rect is defined by top-left and bottom-right Points.
    // These zones correspond to the three possible prop positions on the field.
    // -------------------------------------------------------------------------
    static final Rect LEFT_ROI = new Rect(
            new Point(0, 110),
            new Point(45, 170));
    static final Rect MID_ROI = new Rect(
            new Point(140, 120),
            new Point(190, 160));
    static final Rect RIGHT_ROI = new Rect(
            new Point(280, 100),
            new Point(320, 170));

    // Result field — written by camera thread, read by opmode thread.
    // volatile ensures the opmode thread always reads the latest value.
    public volatile String ObjectDirection;

    // Persistent Mat fields — allocated once at construction, reused every frame.
    // Allocating inside processFrame causes GC pressure and eventual OOM.
    Mat mat    = new Mat();    // HSV converted frame
    Mat thresh = new Mat();    // binary threshold mask

    // Submat views — assigned and released inside processFrame.
    Mat left, right, mid;

    public BluePipeline(Telemetry t, String s) {
        telemetry = t;
        ObjectDirection = s;
    }

    @Override
    public Mat processFrame(Mat input) {

        // Step 1: Convert RGB to HSV.
        // EasyOpenCV delivers frames as RGB (not BGR like standard OpenCV).
        // HSV separates hue from brightness — more robust to lighting changes.
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        // Step 2: Define HSV bounds for blue.
        // H: 80–190 (wider than strict blue 100–130, to tolerate lighting variation)
        // S: 120–255 (ensures meaningful color saturation, rejects washed-out pixels)
        // V: 0–255   (any brightness)
        Scalar lowHSVBlue  = new Scalar(80,  120,   0);
        Scalar highHSVBlue = new Scalar(190, 255, 255);

        // Step 3: Release previous mask content, then compute new binary threshold.
        // thresh becomes 255 where pixel is within blue range, 0 elsewhere.
        thresh.release();
        Core.inRange(mat, lowHSVBlue, highHSVBlue, thresh);

        // Step 4: Extract submat views into each ROI.
        // submat() creates a view — NOT a copy. No pixel data is duplicated.
        // Must be released after use to avoid OpenCV reference counting issues.
        left  = thresh.submat(LEFT_ROI);
        right = thresh.submat(RIGHT_ROI);
        mid   = thresh.submat(MID_ROI);

        // Step 5: Compute density of blue pixels in each ROI.
        // sumElems().val[0] = sum of all pixel values in the Mat.
        // Divide by the ROI's own area() to normalize by region size.
        // Divide by 255 to convert from raw sum to a fraction in [0, 1].
        double leftValue  = Core.sumElems(left).val[0]  / LEFT_ROI.area()  / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;
        double midValue   = Core.sumElems(mid).val[0]   / MID_ROI.area()   / 255;

        // Step 6: Release submat views.
        left.release();
        right.release();
        mid.release();

        // Step 7: Find the region with the highest density.
        double maximum = Math.max(leftValue, Math.max(midValue, rightValue));
        boolean objLeft  = leftValue  == maximum;
        boolean objRight = rightValue == maximum;
        boolean objMid   = midValue   == maximum;

        // Tie-breaking: if two regions share the maximum, resolve by direct comparison.
        if (objLeft && objRight) {
            objLeft  = leftValue > rightValue;
            objRight = !objLeft;
        }
        if (objMid && objRight) {
            objMid   = midValue > rightValue;
            objRight = !objMid;
        }
        if (objLeft && objMid) {
            objLeft = leftValue > midValue;
            objMid  = !objLeft;
        }

        // Step 8: Set result and draw a rectangle on the winning ROI.
        // Drawn on thresh (the binary mask) so it appears in the DS preview.
        if (objLeft) {
            ObjectDirection = "LEFT";
            Imgproc.rectangle(thresh, LEFT_ROI, new Scalar(255, 255, 255), 4);
        } else if (objRight) {
            ObjectDirection = "RIGHT";
            Imgproc.rectangle(thresh, RIGHT_ROI, new Scalar(255, 255, 255), 4);
        } else if (objMid) {
            ObjectDirection = "MIDDLE";
            Imgproc.rectangle(thresh, MID_ROI, new Scalar(255, 255, 255), 4);
        } else {
            ObjectDirection = "NONE";
        }

        // Return the binary mask for the preview.
        // Swap to `return input` to display the raw color feed instead.
        return thresh;
    }

    // Called from the opmode thread to read the current detection result.
    public String getPosition() {
        return ObjectDirection;
    }
}
```

---

### RedPipeline.java — Fixed and Annotated

```java
package org.firstinspires.ftc.teamcode.auto.notshown;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class RedPipeline extends OpenCvPipeline {

    Telemetry telemetry;

    // Same ROI layout as BluePipeline — tuned for a 320x240 frame.
    static final Rect LEFT_ROI = new Rect(
            new Point(0, 110),
            new Point(45, 170));
    static final Rect MID_ROI = new Rect(
            new Point(140, 120),
            new Point(190, 160));
    static final Rect RIGHT_ROI = new Rect(
            new Point(280, 100),
            new Point(320, 170));

    // Result field — volatile for safe cross-thread reads.
    public volatile String ObjectDirection;

    // Persistent Mat fields.
    Mat mat      = new Mat();   // HSV converted frame
    Mat thresh   = new Mat();   // combined binary mask (lowerRed OR upperRed)
    Mat lowerRed = new Mat();   // mask for hue range 0–10
    Mat upperRed = new Mat();   // mask for hue range 170–179

    Mat left, right, mid;

    public RedPipeline(Telemetry t, String s) {
        telemetry = t;
        ObjectDirection = s;
    }

    @Override
    public Mat processFrame(Mat input) {

        // Step 1: Convert RGB to HSV.
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        // Step 2: Threshold for red.
        // Red wraps around in OpenCV HSV — it occupies BOTH the low end (hue 0–10)
        // AND the high end (hue 170–179) of the 0–179 hue scale.
        // Two separate inRange calls are required; their results are OR'd together
        // to produce a single complete mask covering all red pixels.
        Scalar lowHSVRed1  = new Scalar(0,   100, 20);
        Scalar highHSVRed1 = new Scalar(10,  255, 255);
        Scalar lowHSVRed2  = new Scalar(170, 100, 20);
        Scalar highHSVRed2 = new Scalar(179, 255, 255);

        lowerRed.release();
        upperRed.release();
        thresh.release();

        Core.inRange(mat, lowHSVRed1, highHSVRed1, lowerRed);
        Core.inRange(mat, lowHSVRed2, highHSVRed2, upperRed);
        Core.bitwise_or(lowerRed, upperRed, thresh);

        // Step 3: Extract submat views into each ROI.
        left  = thresh.submat(LEFT_ROI);
        right = thresh.submat(RIGHT_ROI);
        mid   = thresh.submat(MID_ROI);

        // Step 4: Compute density, dividing each region by its own area.
        double leftValue  = Core.sumElems(left).val[0]  / LEFT_ROI.area()  / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;
        double midValue   = Core.sumElems(mid).val[0]   / MID_ROI.area()   / 255;

        // Step 5: Release submat views.
        left.release();
        right.release();
        mid.release();

        // Step 6: Find the region with the highest density.
        double maximum = Math.max(leftValue, Math.max(midValue, rightValue));
        boolean objLeft  = leftValue  == maximum;
        boolean objRight = rightValue == maximum;
        boolean objMid   = midValue   == maximum;

        // Tie-breaking — resolves draws by direct comparison.
        if (objLeft && objRight) {
            objLeft  = leftValue > rightValue;
            objRight = !objLeft;
        }
        if (objMid && objRight) {
            objMid   = midValue > rightValue;
            objRight = !objMid;
        }
        if (objLeft && objMid) {
            objLeft = leftValue > midValue;
            objMid  = !objLeft;
        }

        // Step 7: Set result and draw rectangle on the winning ROI.
        // Telemetry is intentionally not called here — that is the opmode's responsibility.
        if (objLeft) {
            ObjectDirection = "LEFT";
            Imgproc.rectangle(thresh, LEFT_ROI, new Scalar(255, 255, 255), 4);
        } else if (objRight) {
            ObjectDirection = "RIGHT";
            Imgproc.rectangle(thresh, RIGHT_ROI, new Scalar(255, 255, 255), 4);
        } else if (objMid) {
            ObjectDirection = "MIDDLE";
            Imgproc.rectangle(thresh, MID_ROI, new Scalar(255, 255, 255), 4);
        } else {
            ObjectDirection = "NONE";
        }

        return thresh;
    }

    public String getPosition() {
        return ObjectDirection;
    }
}
```

---

## Part 2: Camera Initialization Code (Extracted from OpModes)

The following block is identical across all 8 autonomous opmodes. Reproduced once here for clarity.

### Webcam Declaration (class fields)

```java
OpenCvWebcam webcam;
```

### Initialization Block (inside runOpMode, before waitForStart)

```java
// Look up the Android View ID for the DS camera preview surface.
// Omit this and the argument to createWebcam() if no DS preview is needed.
int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
    "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()
);

// Create the webcam instance bound to the hardware config device name "Webcam 1".
webcam = OpenCvCameraFactory.getInstance().createWebcam(
    hardwareMap.get(WebcamName.class, "Webcam 1"),
    cameraMonitorViewId
);

// Construct the pipeline.
// For Blue autonomous: new BluePipeline(telemetry, null)
// For Red autonomous:  new RedPipeline(telemetry, null)
BluePipeline pipeline = new BluePipeline(telemetry, null);

// Register the pipeline. Frames are sent to pipeline.processFrame() each tick.
webcam.setPipeline(pipeline);

// Set timeout for Android USB camera permission dialog.
webcam.setMillisecondsPermissionTimeout(5000);

// Open camera asynchronously. startStreaming must be called inside onOpened.
webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
    @Override
    public void onOpened() {
        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
    }

    @Override
    public void onError(int errorCode) {
        // Camera failed to open. Detection will fall back to the default randomization value.
        telemetry.addData("[CAMERA ERROR]", errorCode);
        telemetry.update();
    }
});
```

### Init Loop (polls detection before match start)

```java
while (opModeInInit()) {
    telemetry.addData("Frame Count",        webcam.getFrameCount());
    telemetry.addData("FPS",                String.format("%.2f", webcam.getFps()));
    telemetry.addData("Total frame time ms",webcam.getTotalFrameTimeMs());
    telemetry.addData("Pipeline time ms",   webcam.getPipelineTimeMs());
    telemetry.addData("Overhead time ms",   webcam.getOverheadTimeMs());
    telemetry.addData("Theoretical max FPS",webcam.getCurrentPipelineMaxFps());
    telemetry.update();

    // Continuously update the randomization enum from the pipeline's latest result.
    // The value set on the last iteration before Start is pressed determines the path.
    if (Objects.equals(pipeline.getPosition(), "LEFT")) {
        randomization = PropPosition.LEFT;
    } else if (Objects.equals(pipeline.getPosition(), "MIDDLE")) {
        randomization = PropPosition.MIDDLE;
    } else if (Objects.equals(pipeline.getPosition(), "RIGHT")) {
        randomization = PropPosition.RIGHT;
    }

    sleep(100); // 10 Hz polling — sufficient since the prop is stationary
}
```

### Camera Close (before match run begins)

```java
// Camera is closed after init because prop detection is no longer needed during the match.
// This frees USB bandwidth and Android resources.
webcam.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener() {
    @Override
    public void onClose() {}
});
```

### Red Alliance — LEFT/RIGHT Inversion

In all Red opmodes, LEFT and RIGHT are swapped because the Red robot is on the mirrored side of the field:

```java
if (Objects.equals(pipeline.getPosition(), "LEFT")) {
    randomization = PropPosition.RIGHT; // intentionally inverted
} else if (Objects.equals(pipeline.getPosition(), "MIDDLE")) {
    randomization = PropPosition.MIDDLE;
} else if (Objects.equals(pipeline.getPosition(), "RIGHT")) {
    randomization = PropPosition.LEFT;  // intentionally inverted
}
```

---

## Part 3: Sample OpMode — Camera Only

A minimal, self-contained opmode demonstrating the complete camera pipeline pattern for detecting either a red or blue target across three fixed positions. No drive, intake, or outtake dependencies. Safe to deploy and run standalone for tuning and testing.

```java
package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;
import java.util.Objects;

/**
 * CameraOnlyAuto
 *
 * Standalone opmode demonstrating the full camera pipeline pattern.
 * Detects a red or blue target at one of three positions (LEFT, MIDDLE, RIGHT).
 *
 * Usage:
 *   1. Set ALLIANCE = Alliance.BLUE or Alliance.RED before deployment.
 *   2. Deploy and run. Watch DS telemetry during init.
 *   3. Press Start — the opmode logs the final detection result and stops.
 *
 * Tuning:
 *   - Adjust ROI pixel coordinates in TargetPipeline to match your camera mount.
 *   - Adjust HSV bounds if detection is unreliable under competition lighting.

 *   - Temporarily change `return input` to `return mask` in processFrame to inspect
 *     the raw binary threshold while tuning HSV values.
 */
@Autonomous(name = "CameraOnlyAuto")
public class CameraOnlyAuto extends LinearOpMode {

    // -------------------------------------------------------------------------
    // Configuration
    // -------------------------------------------------------------------------

    enum Alliance { BLUE, RED }

    // Change this to match your alliance before deploying.
    // Blue: no LEFT/RIGHT inversion.
    // Red: LEFT and RIGHT are swapped to account for field mirroring.
    public static Alliance ALLIANCE = Alliance.BLUE;

    // Camera resolution. 320x240 is standard for 3-zone ROI detection.
    // Higher resolutions provide more detail but increase pipeline time.
    public static int CAMERA_WIDTH  = 320;
    public static int CAMERA_HEIGHT = 240;

    // -------------------------------------------------------------------------
    // Instances
    // -------------------------------------------------------------------------

    OpenCvWebcam webcam;
    TargetPipeline pipeline;

    enum TargetPosition { LEFT, MIDDLE, RIGHT, UNKNOWN }
    TargetPosition detectedPosition = TargetPosition.UNKNOWN;

    // -------------------------------------------------------------------------
    // runOpMode
    // -------------------------------------------------------------------------

    @Override
    public void runOpMode() throws InterruptedException {



        // Enable bulk reads on all hubs to reduce I2C overhead if sensors are present.
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO));

        // ----- Camera Initialization -----

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()
        );

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                cameraMonitorViewId
        );

        pipeline = new TargetPipeline(ALLIANCE);
        webcam.setPipeline(pipeline);



        webcam.setMillisecondsPermissionTimeout(5000);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("[CAMERA ERROR]", "Code: " + errorCode);
                telemetry.update();
                // detectedPosition remains UNKNOWN; the run phase handles this with a safe default.
            }
        });

        // ----- Init Loop -----

        telemetry.addLine("Waiting for start — watching for target...");
        telemetry.update();

        while (opModeInInit()) {
            String rawPosition = pipeline.getPosition();

            // Map raw pipeline string to the TargetPosition enum.
            // For Red alliance, LEFT and RIGHT are inverted to account for field mirroring.
            if (ALLIANCE == Alliance.RED) {
                if      (Objects.equals(rawPosition, "LEFT"))   detectedPosition = TargetPosition.RIGHT;
                else if (Objects.equals(rawPosition, "MIDDLE")) detectedPosition = TargetPosition.MIDDLE;
                else if (Objects.equals(rawPosition, "RIGHT"))  detectedPosition = TargetPosition.LEFT;
                else                                             detectedPosition = TargetPosition.UNKNOWN;
            } else {
                if      (Objects.equals(rawPosition, "LEFT"))   detectedPosition = TargetPosition.LEFT;
                else if (Objects.equals(rawPosition, "MIDDLE")) detectedPosition = TargetPosition.MIDDLE;
                else if (Objects.equals(rawPosition, "RIGHT"))  detectedPosition = TargetPosition.RIGHT;
                else                                             detectedPosition = TargetPosition.UNKNOWN;
            }

            telemetry.addData("Alliance",         ALLIANCE.toString());
            telemetry.addData("Raw pipeline",     rawPosition);
            telemetry.addData("Mapped position",  detectedPosition.toString());
            telemetry.addLine("---");
            telemetry.addData("FPS",              String.format("%.2f", webcam.getFps()));
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Frame count",      webcam.getFrameCount());
            telemetry.update();

            sleep(100);
        }

        // ----- Close Camera -----

        webcam.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener() {
            @Override
            public void onClose() {}
        });

        // ----- Run Phase -----

        waitForStart();
        if (!opModeIsActive()) return;

        telemetry.addData("FINAL DETECTION", detectedPosition.toString());
        telemetry.update();

        switch (detectedPosition) {
            case LEFT:
                telemetry.addLine("Executing LEFT path");
                telemetry.update();
                // drive.followPath(leftPath);
                break;

            case MIDDLE:
                telemetry.addLine("Executing MIDDLE path");
                telemetry.update();
                // drive.followPath(middlePath);
                break;

            case RIGHT:
                telemetry.addLine("Executing RIGHT path");
                telemetry.update();
                // drive.followPath(rightPath);
                break;

            case UNKNOWN:
            default:
                // Camera failed or target never detected.
                // Default to RIGHT — change this to whatever your safe fallback path is.
                telemetry.addLine("WARNING: No detection. Defaulting to RIGHT.");
                telemetry.update();
                // drive.followPath(defaultPath);
                break;
        }

        sleep(2000);
    }


    // =========================================================================
    // TargetPipeline
    //
    // Unified red/blue detection pipeline. Alliance is passed at construction.
    //
    // Blue: single inRange pass on hue 100–130.
    // Red: two inRange passes (hue 0–10 and 170–179) OR'd together — correctly
    //      handles red's wraparound at both ends of the OpenCV HSV hue scale.
    //
    // Detection method: ROI density (fraction of target-colored pixels per zone).
    // The zone with the highest density is reported as the detected position.
    //
    // Frame: 320x240 RGB input from EasyOpenCV.
    // Output: Annotated RGB frame with ROI outlines and density values overlaid.
    //         Swap to `return mask` in processFrame to inspect the binary threshold.
    // =========================================================================

    static class TargetPipeline extends OpenCvPipeline {

        // ----- ROI Definitions -----
        // Pixel coordinates for a 320x240 frame.
        // Adjust these based on where the target appears for your specific camera mount.
        // Each ROI should comfortably contain the target with ~10–20px margin.
        static final Rect LEFT_ROI  = new Rect(new Point(0,   110), new Point(45,  170));
        static final Rect MID_ROI   = new Rect(new Point(140, 120), new Point(190, 160));
        static final Rect RIGHT_ROI = new Rect(new Point(280, 100), new Point(320, 170));

        // Minimum density to count as a valid detection.
        // Prevents noise or ambient color from winning when no target is present.
        static final double MIN_DENSITY_THRESHOLD = 0.02;

        // ----- HSV Bounds -----
        // Blue: single range. Hue 100–130 covers blue well under most lighting.
        static final Scalar BLUE_LOW  = new Scalar(100, 120,  50);
        static final Scalar BLUE_HIGH = new Scalar(130, 255, 255);

        // Red: two ranges. Hue wraps — 0–10 captures warm-shifted red,
        // 170–179 captures cool-shifted red. Both are needed for reliable detection.
        static final Scalar RED_LOW_1  = new Scalar(0,   100,  20);
        static final Scalar RED_HIGH_1 = new Scalar(10,  255, 255);
        static final Scalar RED_LOW_2  = new Scalar(170, 100,  20);
        static final Scalar RED_HIGH_2 = new Scalar(179, 255, 255);

        // ----- Alliance -----
        final CameraOnlyAuto.Alliance alliance;

        // ----- Persistent Mat fields -----
        // Allocated once, reused every frame to avoid GC pressure.
        Mat hsv     = new Mat();   // HSV converted input frame
        Mat mask    = new Mat();   // final binary mask
        Mat redLow  = new Mat();   // red lower hue range mask (0–10)
        Mat redHigh = new Mat();   // red upper hue range mask (170–179)

        // ----- Result -----
        // volatile for safe single-writer (camera thread) / single-reader (opmode thread) access.
        volatile String position = "NONE";

        TargetPipeline(CameraOnlyAuto.Alliance alliance) {
            this.alliance = alliance;
        }

        @Override
        public Mat processFrame(Mat input) {

            // Step 1: Convert RGB to HSV.
            // EasyOpenCV delivers RGB frames — use COLOR_RGB2HSV, not COLOR_BGR2HSV.
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            // Step 2: Threshold to isolate target color.
            mask.release();

            if (alliance == CameraOnlyAuto.Alliance.BLUE) {
                Core.inRange(hsv, BLUE_LOW, BLUE_HIGH, mask);
            } else {
                // Red wraps around the hue circle. Two passes are required.
                // Lower range: hue 0–10  (red shifted warm/yellow)
                // Upper range: hue 170–179 (red shifted cool/magenta)
                redLow.release();
                redHigh.release();
                Core.inRange(hsv, RED_LOW_1, RED_HIGH_1, redLow);
                Core.inRange(hsv, RED_LOW_2, RED_HIGH_2, redHigh);
                Core.bitwise_or(redLow, redHigh, mask);
            }

            // Step 3: Sample density in each ROI.
            // Each region is divided by its own area — not a shared constant.
            Mat roiLeft  = mask.submat(LEFT_ROI);
            Mat roiMid   = mask.submat(MID_ROI);
            Mat roiRight = mask.submat(RIGHT_ROI);

            double leftDensity  = Core.sumElems(roiLeft).val[0]  / LEFT_ROI.area()  / 255.0;
            double midDensity   = Core.sumElems(roiMid).val[0]   / MID_ROI.area()   / 255.0;
            double rightDensity = Core.sumElems(roiRight).val[0] / RIGHT_ROI.area() / 255.0;

            roiLeft.release();
            roiMid.release();
            roiRight.release();

            // Step 4: Classify by highest density, with minimum threshold check.
            double maxDensity = Math.max(leftDensity, Math.max(midDensity, rightDensity));

            if (maxDensity < MIN_DENSITY_THRESHOLD) {
                position = "NONE";
            } else if (leftDensity == maxDensity) {
                position = "LEFT";
            } else if (midDensity == maxDensity) {
                position = "MIDDLE";
            } else {
                position = "RIGHT";
            }

            // Step 5: Draw diagnostic overlays on the input frame.
            // All three ROI outlines drawn in gray; winning ROI drawn in white.
            Imgproc.rectangle(input, LEFT_ROI,  new Scalar(150, 150, 150), 2);
            Imgproc.rectangle(input, MID_ROI,   new Scalar(150, 150, 150), 2);
            Imgproc.rectangle(input, RIGHT_ROI, new Scalar(150, 150, 150), 2);

            if (!position.equals("NONE")) {
                Rect winner = position.equals("LEFT")   ? LEFT_ROI :
                              position.equals("MIDDLE") ? MID_ROI  : RIGHT_ROI;
                Imgproc.rectangle(input, winner, new Scalar(255, 255, 255), 4);
            }

            // Detection result label
            Imgproc.putText(input, position, new Point(5, 20),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, new Scalar(255, 255, 255), 2);

            // Per-ROI density values — useful for tuning ROI positions and HSV bounds
            Imgproc.putText(input, String.format("L:%.2f", leftDensity),
                    new Point(0, CAMERA_HEIGHT - 30),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.4, new Scalar(255, 255, 255), 1);
            Imgproc.putText(input, String.format("M:%.2f", midDensity),
                    new Point(110, CAMERA_HEIGHT - 30),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.4, new Scalar(255, 255, 255), 1);
            Imgproc.putText(input, String.format("R:%.2f", rightDensity),
                    new Point(240, CAMERA_HEIGHT - 30),
                    Imgproc.FONT_HERSHEY_SIMPLEX, 0.4, new Scalar(255, 255, 255), 1);

            // Return the annotated RGB frame.
            // Swap to `return mask` to inspect the raw binary threshold during HSV tuning.
            return input;
        }

        public String getPosition() {
            return position;
        }
    }
}
```

---

## Part 4: Comparison — Original vs Fixed

| Aspect | Original BluePipeline | Fixed BluePipeline |
|---|---|---|
| `midValue` area divisor | `RIGHT_ROI.area()` — wrong | `MID_ROI.area()` — correct |
| `volatile` on result field | Missing | Added |
| Tie-breaking | Present | Unchanged |

| Aspect | Original RedPipeline | Fixed RedPipeline |
|---|---|---|
| `midValue` area divisor | `RIGHT_ROI.area()` — wrong | `MID_ROI.area()` — correct |
| Red hue coverage | Hue 0–13 only — misses 165–179 | Hue 0–10 AND 170–179 OR'd — complete |
| Dead fields `upperthresh` / `lowerthresh` | Declared, never written to | Removed; replaced with `redLow` / `redHigh` |
| Tie-breaking | Missing — left wins all ties silently | Added — matches BluePipeline behavior |
| `telemetry.addData` in `processFrame` | Present — wrong layer | Removed |
| `volatile` on result field | Missing | Added |

---

## Part 5: HSV Tuning Procedure

When detection is unreliable, use this procedure to retune the HSV bounds:

1. Deploy `CameraOnlyAuto` and start the opmode.
2. In `processFrame`, temporarily change `return input` to `return mask`.
3. Place the target at each of the three positions and observe the binary mask.
4. The target region should appear predominantly white. The background should be black.
5. If the target is partially or fully black — bounds are too narrow; widen the hue or saturation range.
6. If too much background is white — bounds are too wide; narrow them.
7. For red: verify both hue ranges contribute by temporarily commenting out one `inRange` call at a time.
8. Once the threshold is clean, revert to `return input`.
