# FTC EasyOpenCV Camera Pipeline Reference

This document is a complete reference for building OpenCV-based vision pipelines in FTC using EasyOpenCV. It is intended for use by a developer or AI to construct any camera pipeline from scratch.

---

## Dependencies

These must be present in your `build.gradle` or `build.dependencies.gradle`:

```groovy
implementation 'org.openftc:easyopencv:1.7.3'          // EasyOpenCV
```

EasyOpenCV bundles OpenCV for Android — you do not import OpenCV separately.

---

## Core Imports

Every pipeline and opmode using the camera will need some subset of these:

```java
// EasyOpenCV
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

// FTC hardware
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

// OpenCV operations
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

// Standard FTC
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
```

---

## Architecture Overview

```
LinearOpMode
    │
    ├── Declare:  OpenCvWebcam webcam
    ├── Init:     create webcam → set pipeline → open async → start streaming
    ├── Init loop: poll pipeline.getResult() each tick
    ├── On Start: close camera (if no longer needed)
    └── Run:      use detection result

OpenCvPipeline (subclass)
    ├── Fields:   Mat objects for processing, result variable
    ├── processFrame(Mat input): called per frame on camera thread
    │       ├── color conversion
    │       ├── thresholding / filtering
    │       ├── ROI sampling or contour detection
    │       ├── result classification
    │       └── return frame to display
    └── getResult(): called from opmode thread to read detection
```

---

## Pipeline Class Structure

A pipeline is a class that extends `OpenCvPipeline` and overrides `processFrame`. All image processing happens inside `processFrame`, which is called on a dedicated camera thread.

```java
public class MyPipeline extends OpenCvPipeline {

    // --- Persistent Mat objects (allocate once, reuse every frame) ---
    Mat mat  = new Mat();    // intermediate processing buffer
    Mat mask = new Mat();    // binary threshold result

    // --- Result storage (written by camera thread, read by opmode thread) ---
    private volatile String result = "NONE";

    // --- Constructor ---
    public MyPipeline() {
        // initialize any constants here
    }

    // --- Called once per camera frame ---
    @Override
    public Mat processFrame(Mat input) {
        // ... processing here ...
        return input; // or return a processed Mat for the preview
    }

    // --- Called from opmode thread ---
    public String getResult() {
        return result;
    }
}
```

**Thread safety:** `processFrame` runs on the camera thread. Your opmode reads results on the main thread. Mark shared result fields `volatile` for safe single-writer single-reader access. For complex result objects, use `synchronized` or `AtomicReference`.

---

## Color Spaces

### RGB vs HSV

Always convert to HSV before thresholding by color. HSV separates hue (color identity) from saturation (color intensity) and value (brightness), making detection robust against lighting variation.

```java
Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
// Note: EasyOpenCV delivers frames as RGB, not BGR (unlike standard OpenCV).
// Use COLOR_RGB2HSV, not COLOR_BGR2HSV.
```

### OpenCV HSV Ranges

OpenCV uses a compressed HSV range: H is 0–179 (not 0–360), S is 0–255, V is 0–255.

| Color  | Hue Range (OpenCV) | Notes |
|--------|--------------------|-------|
| Red    | 0–10 AND 170–179   | Wraps around — needs two ranges OR'd together |
| Orange | 10–20              | |
| Yellow | 20–35              | |
| Green  | 35–85              | |
| Cyan   | 85–100             | |
| Blue   | 100–130            | |
| Purple | 130–160            | |
| Pink   | 160–170            | |

### Handling Red (Hue Wraparound)

Red spans both ends of the hue circle. Capture it correctly with two `inRange` calls:

```java
Mat lowerRed = new Mat();
Mat upperRed = new Mat();

Core.inRange(hsvMat, new Scalar(0, 100, 20),   new Scalar(10, 255, 255),  lowerRed);
Core.inRange(hsvMat, new Scalar(170, 100, 20),  new Scalar(179, 255, 255), upperRed);

Core.bitwise_or(lowerRed, upperRed, mask);

lowerRed.release();
upperRed.release();
```

Omitting the upper range causes missed detections when the prop's hue sits above 170 (common under warm lighting).

---

## Thresholding

`Core.inRange` produces a binary mask: 255 where the pixel is within the scalar bounds, 0 elsewhere.

```java
// Single range
Core.inRange(hsvMat, lowerBound, upperBound, mask);

// Two ranges OR'd (red)
Core.inRange(hsvMat, low1, high1, mask1);
Core.inRange(hsvMat, low2, high2, mask2);
Core.bitwise_or(mask1, mask2, mask);
```

### Noise Reduction (Optional but Recommended)

After thresholding, small spurious blobs can be eliminated with morphological operations:

```java
Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));

// Erode removes small noise
Imgproc.erode(mask, mask, kernel);

// Dilate restores object size after erosion
Imgproc.dilate(mask, mask, kernel);

// Combined: erode then dilate = morphological opening (removes small blobs)
Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);

// Dilate then erode = morphological closing (fills small holes)
Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);
```

---

## Detection Methods

### Method A: Region of Interest (ROI) Density

Best for fixed-position detection where you know approximately where the object will appear (e.g., prop position detection with 3 candidate zones).

```java
// Define ROI as a Rect(topLeftPoint, bottomRightPoint)
static final Rect LEFT_ROI   = new Rect(new Point(0,   110), new Point(45,  170));
static final Rect MIDDLE_ROI = new Rect(new Point(140, 120), new Point(190, 160));
static final Rect RIGHT_ROI  = new Rect(new Point(280, 100), new Point(320, 170));

// Sample density inside each ROI
Mat roiLeft   = mask.submat(LEFT_ROI);
Mat roiMid    = mask.submat(MIDDLE_ROI);
Mat roiRight  = mask.submat(RIGHT_ROI);

// Density = fraction of white pixels in the region
double leftDensity   = Core.sumElems(roiLeft).val[0]  / LEFT_ROI.area()   / 255.0;
double midDensity    = Core.sumElems(roiMid).val[0]   / MIDDLE_ROI.area() / 255.0;
double rightDensity  = Core.sumElems(roiRight).val[0] / RIGHT_ROI.area()  / 255.0;

// ALWAYS release submat views when done
roiLeft.release();
roiMid.release();
roiRight.release();

// Classify by highest density
if (leftDensity >= midDensity && leftDensity >= rightDensity) {
    result = "LEFT";
} else if (midDensity >= rightDensity) {
    result = "MIDDLE";
} else {
    result = "RIGHT";
}
```

**Choosing ROI coordinates:** Observe where the prop appears in the 320x240 frame for each position. Set ROI boxes to cover those zones without overlapping. Leave margin — don't cut the ROIs so tight that slight robot placement variation moves the prop out of the ROI.

**Common mistake:** Using the same divisor (e.g., `RIGHT_ROI.area()`) for all three regions instead of each region's own area. This skews density comparisons when ROIs differ in size.

---

### Method B: Contour Detection

Best for when the object's position is not known in advance, or when you need bounding box, centroid, or size information.

```java
// Find contours in the binary mask
List<MatOfPoint> contours = new ArrayList<>();
Mat hierarchy = new Mat();
Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
hierarchy.release();

// Find the largest contour by area
double maxArea = 0;
Rect largestBoundingBox = null;

for (MatOfPoint contour : contours) {
    double area = Imgproc.contourArea(contour);
    if (area > maxArea) {
        maxArea = area;
        largestBoundingBox = Imgproc.boundingRect(contour);
    }
}

// Release contour Mats
for (MatOfPoint c : contours) c.release();

// Use result
if (largestBoundingBox != null && maxArea > MIN_AREA_THRESHOLD) {
    // centroid
    double cx = largestBoundingBox.x + largestBoundingBox.width  / 2.0;
    double cy = largestBoundingBox.y + largestBoundingBox.height / 2.0;
    result = "DETECTED at (" + (int)cx + ", " + (int)cy + ")";
} else {
    result = "NONE";
}
```

**Filtering contours:** Set a minimum area threshold (e.g., 500 pixels) to ignore noise:
```java
static final double MIN_AREA_THRESHOLD = 500.0;
```

---

### Method C: Aspect Ratio / Shape Filtering

After finding contours, filter by shape to distinguish target objects from noise:

```java
for (MatOfPoint contour : contours) {
    Rect bbox = Imgproc.boundingRect(contour);
    double aspectRatio = (double) bbox.width / bbox.height;
    double area = Imgproc.contourArea(contour);

    // Example: only accept roughly square blobs with sufficient area
    if (area > 500 && aspectRatio > 0.5 && aspectRatio < 2.0) {
        // valid detection
    }
}
```

---

## Drawing on the Output Frame

Draw diagnostic overlays on the Mat you return from `processFrame` to visualize what the pipeline is detecting:

```java
// Rectangle around ROI
Imgproc.rectangle(outputMat, roi, new Scalar(255, 255, 255), 4);
// Args: mat, Rect, color (BGR or grayscale Scalar), thickness in pixels

// Rectangle around bounding box
Imgproc.rectangle(outputMat, boundingBox, new Scalar(0, 255, 0), 2);

// Circle at centroid
Imgproc.circle(outputMat, new Point(cx, cy), 8, new Scalar(255, 0, 0), -1);
// -1 thickness = filled circle

// Text label
Imgproc.putText(outputMat, "LEFT", new Point(10, 30),
    Imgproc.FONT_HERSHEY_SIMPLEX, 1.0, new Scalar(255, 255, 255), 2);
```

**What to return:** Return the frame you want to appear in the DS preview. Options:
- `return input` — raw RGB frame with overlays drawn on it
- `return mask` — binary threshold mask (black and white; easier to verify thresholding)
- `return mat` — HSV frame (useful for debugging color ranges)

To draw on `input` without destroying it for further processing, clone it first:
```java
Mat display = input.clone();
// ... draw on display ...
return display; // caller takes ownership; release not needed here
```

---

## Memory Management

EasyOpenCV runs on Android with limited heap. Improper Mat handling causes memory leaks and eventual crashes.

### Rules

1. **Allocate persistent Mats as fields**, not inside `processFrame`. Field Mats are allocated once and reused.
2. **Call `.release()` on temporary Mats** created inside `processFrame` before the method returns.
3. **Call `.release()` on submat views** after you are done sampling them. A submat is a view, not a copy — releasing it does not free memory, but it is required to avoid OpenCV reference counting issues.
4. **Call `.release()` before reusing a field Mat** if the Mat is being reassigned (e.g., `thresh.release(); Core.inRange(...)` recomputes into `thresh`).
5. **Never release `input`** — EasyOpenCV manages the input frame's lifecycle.

```java
// CORRECT pattern
public class MyPipeline extends OpenCvPipeline {
    Mat hsv   = new Mat();  // field — allocated once
    Mat mask  = new Mat();  // field — allocated once

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);  // reuses hsv field

        mask.release();  // release before Core.inRange writes into it
        Core.inRange(hsv, lower, upper, mask);

        Mat roi = mask.submat(SOME_RECT);  // temporary submat view
        double density = Core.sumElems(roi).val[0] / SOME_RECT.area() / 255.0;
        roi.release();   // release the view

        return input;
    }
}
```

---

## Camera Setup in OpMode — Full Sequence

### Step 1: Declare

```java
OpenCvWebcam webcam;
MyPipeline pipeline;
```

### Step 2: Initialize (inside `runOpMode`, before `waitForStart`)

```java
// Get DS preview surface ID
int cameraMonitorViewId = hardwareMap.appContext.getResources()
    .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

// Create webcam instance bound to hardware config name "Webcam 1"
webcam = OpenCvCameraFactory.getInstance()
    .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

// Construct and assign pipeline
pipeline = new MyPipeline();
webcam.setPipeline(pipeline);

// Set USB permission timeout
webcam.setMillisecondsPermissionTimeout(5000);

// Open camera asynchronously
webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
    @Override
    public void onOpened() {
        // Resolution: 320x240 is standard. Higher = more CPU, more detail.
        // Rotation: UPRIGHT if webcam faces forward right-side-up.
        //           SIDEWAYS_LEFT / SIDEWAYS_RIGHT if mounted rotated.
        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
    }

    @Override
    public void onError(int errorCode) {
        telemetry.addData("Camera error", errorCode);
        telemetry.update();
    }
});
```

### Step 3: Init Loop — Poll Detection

```java
while (opModeInInit()) {
    telemetry.addData("Detection", pipeline.getResult());
    telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
    telemetry.update();
    sleep(100);
}
```

### Step 4: Close Camera When No Longer Needed

```java
// If detection is only needed before the match starts:
webcam.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener() {
    @Override
    public void onClose() {}
});
```

If you need the camera during the match (e.g., for object tracking during teleop), do not close it.

### Step 5: Use Detection Result

```java
waitForStart();
String detected = pipeline.getResult(); // read final value

if (detected.equals("LEFT")) {
    // path A
} else if (detected.equals("MIDDLE")) {
    // path B
} else {
    // path C
}
```

---

## Resolution and Rotation Options

### Resolution

```java
webcam.startStreaming(width, height, rotation);
```

Common choices:

| Resolution | Use Case |
|------------|----------|
| 320×240    | Standard prop detection — low CPU, sufficient detail |
| 640×480    | More detail for contour detection or AprilTags |
| 1280×720   | High detail — significant CPU cost on Control Hub |

Lower resolution reduces pipeline time. Always check `webcam.getPipelineTimeMs()` in telemetry to verify the pipeline is keeping up.

### Rotation Constants

```java
OpenCvCameraRotation.UPRIGHT        // no rotation — camera faces forward, top edge up
OpenCvCameraRotation.SIDEWAYS_LEFT  // 90° CCW rotation correction
OpenCvCameraRotation.SIDEWAYS_RIGHT // 90° CW rotation correction
OpenCvCameraRotation.UPSIDE_DOWN    // 180° rotation correction
```

These correct for how the camera is physically mounted, not how the robot is oriented.

---

## Telemetry Diagnostics

Standard telemetry fields to include during tuning:

```java
telemetry.addData("Frame Count",        webcam.getFrameCount());
telemetry.addData("FPS",                String.format("%.2f", webcam.getFps()));
telemetry.addData("Pipeline time ms",   webcam.getPipelineTimeMs());
telemetry.addData("Overhead time ms",   webcam.getOverheadTimeMs());
telemetry.addData("Total frame time ms",webcam.getTotalFrameTimeMs());
telemetry.addData("Max possible FPS",   webcam.getCurrentPipelineMaxFps());
```

**Interpreting values:**
- `Pipeline time ms`: time your `processFrame` takes. Keep under ~33ms for 30 FPS.
- `Overhead time ms`: EasyOpenCV internal cost — not controllable.
- `Total frame time ms` = pipeline + overhead.
- `Max possible FPS` = 1000 / total frame time.

---





## Quick Reference: Common Scalars

```java
// HSV bounds — Blue
Scalar BLUE_LOW  = new Scalar(100, 120,  50);
Scalar BLUE_HIGH = new Scalar(130, 255, 255);

// HSV bounds — Red (lower range)
Scalar RED_LOW_1  = new Scalar(0,   100, 20);
Scalar RED_HIGH_1 = new Scalar(10,  255, 255);

// HSV bounds — Red (upper range, must OR with lower)
Scalar RED_LOW_2  = new Scalar(170, 100, 20);
Scalar RED_HIGH_2 = new Scalar(179, 255, 255);

// Draw colors (for Imgproc drawing calls — these are BGR since drawing is on the output)
Scalar WHITE  = new Scalar(255, 255, 255);
Scalar GREEN  = new Scalar(0,   255, 0);
Scalar RED    = new Scalar(255, 0,   0);
Scalar BLUE   = new Scalar(0,   0,   255);
```

---

## Checklist: Building a New Pipeline

- [ ] Extend `OpenCvPipeline`, override `processFrame`
- [ ] Allocate `Mat` fields at class level, not inside `processFrame`
- [ ] Convert to HSV with `COLOR_RGB2HSV` (not BGR)
- [ ] For red: use two `inRange` calls + `bitwise_or`
- [ ] Release submat views after use
- [ ] Release field Mats before reuse (`mask.release()` before `Core.inRange`)
- [ ] Mark result fields `volatile`
- [ ] Provide `getResult()` accessor
- [ ] Set `cameraMonitorViewId` if DS preview is needed
- [ ] Call `openCameraDeviceAsync` and start streaming in `onOpened`
- [ ] Poll `pipeline.getResult()` in init loop
- [ ] Close camera when detection is complete
- [ ] Verify correct ROI area divisors if using density method
- [ ] Test at actual competition lighting — retune HSV bounds if necessary
