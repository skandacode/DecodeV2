package org.firstinspires.ftc.teamcode.subsystems;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.imgproc.Moments;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

/**
 * Detects largest connected patch that is either green OR purple.
 * Input frame is RGB from EasyOpenCV.
 */
public class ArtifactPipeline extends OpenCvPipeline {

    // HSV thresholds provided by user
    private static final Scalar GREEN_LOW = new Scalar(78, 123, 44);
    private static final Scalar GREEN_HIGH = new Scalar(104, 255, 255);

    private static final Scalar PURPLE_LOW = new Scalar(119, 123, 44);
    private static final Scalar PURPLE_HIGH = new Scalar(146, 255, 255);

    // Minimum area to accept as a valid clump (tune as needed)
    private static final double MIN_AREA = 1500.0;

    // Reused mats
    private final Mat hsv = new Mat();
    private final Mat greenMask = new Mat();
    private final Mat purpleMask = new Mat();
    private final Mat unionMask = new Mat();
    private final Mat morphKernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new org.opencv.core.Size(40, 90));
    private final Mat hierarchy = new Mat();

    // Result fields shared with opmode thread
    private volatile boolean found = false;
    private volatile int centerX = -1;
    private volatile int centerY = -1;
    private volatile double bestArea = 0.0;

    @Override
    public Mat processFrame(Mat input) {
        // EasyOpenCV gives RGB frames
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        // Threshold green and purple independently
        Core.inRange(hsv, GREEN_LOW, GREEN_HIGH, greenMask);
        Core.inRange(hsv, PURPLE_LOW, PURPLE_HIGH, purpleMask);

        // Union: any green OR purple pixel
        Core.bitwise_or(greenMask, purpleMask, unionMask);

        // Clump nearby pixels into one blob (close operation)
        Imgproc.morphologyEx(unionMask, unionMask, Imgproc.MORPH_CLOSE, morphKernel);

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(unionMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        MatOfPoint bestContour = null;
        double localBestArea = MIN_AREA;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > localBestArea) {
                localBestArea = area;
                bestContour = contour;
            }
        }

        if (bestContour != null) {
            Moments m = Imgproc.moments(bestContour);
            if (m.get_m00() != 0.0) {
                int cx = (int) (m.get_m10() / m.get_m00());
                int cy = (int) (m.get_m01() / m.get_m00());

                found = true;
                centerX = cx;
                centerY = cy;
                bestArea = localBestArea;

                // Draw diagnostics
                Imgproc.drawContours(input, contours, contours.indexOf(bestContour), new Scalar(0, 255, 255), 2);
                Imgproc.circle(input, new Point(cx, cy), 6, new Scalar(255, 255, 255), -1);
                Imgproc.putText(input, "X:" + cx + " Y:" + cy, new Point(10, 20),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.6, new Scalar(255, 255, 255), 2);
            } else {
                found = false;
                centerX = -1;
                centerY = -1;
                bestArea = 0.0;
            }
        } else {
            found = false;
            centerX = -1;
            centerY = -1;
            bestArea = 0.0;
        }

        for (MatOfPoint contour : contours) {
            contour.release();
        }

        // Return annotated RGB preview
        return input;
    }

    public boolean isFound() {
        return found;
    }

    public int getCenterX() {
        return centerX;
    }

    public int getCenterY() {
        return centerY;
    }

    public double getBestArea() {
        return bestArea;
    }
}
