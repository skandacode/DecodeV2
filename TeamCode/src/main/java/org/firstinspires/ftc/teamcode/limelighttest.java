package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class limelighttest extends LinearOpMode {
    Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        limelight.pipelineSwitch(0);
        String pythonCode = "import cv2\n" +
                "import numpy as np\n" +
                "import itertools\n" +
                "\n" +
                "def runPipeline(image, llrobot):\n" +
                "    llpython = np.array([0.0]*8)  # 8 floats\n" +
                "\n" +
                "    #cv2.rectangle(image, (0,0), (160,480), (0,0,0), -1)\n" +
                "    cv2.rectangle(image, (0,0), (640,240), (0,0,0), -1)\n" +
                "\n" +
                "\n" +
                "    img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)\n" +
                "    \n" +
                "\n" +
                "    \n" +
                "    green_lower = np.array([65,190, 65])\n" +
                "    green_upper = np.array([91, 255, 255])\n" +
                "    purple_lower = np.array([112, 120, 40])\n" +
                "    purple_upper = np.array([135, 255, 190])\n" +
                "    \n" +
                "    green_mask = cv2.inRange(img_hsv, green_lower, green_upper)\n" +
                "    purple_mask = cv2.inRange(img_hsv, purple_lower, purple_upper)\n" +
                "    green_mask = cv2.bitwise_or(green_mask, purple_mask)\n" +
                "   \n" +
                "    \n" +
                "    kernel = np.ones((13, 13), np.uint8)\n" +
                "    green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel)\n" +
                "    kernel = np.ones((13,13), np.uint8)\n" +
                "    green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)\n" +
                "    kernel = np.ones((13, 13), np.uint8) \n" +
                "    green_mask = cv2.erode(green_mask, kernel, iterations=1)\n" +
                "    kernel = np.ones((7,7), np.uint8) \n" +
                "    green_mask = cv2.dilate(green_mask, kernel, iterations=3)\n" +
                "    mask = green_mask\n" +
                "\n" +
                "    #dist = cv2.distanceTransform(mask, cv2.DIST_L2, 5)\n" +
                "\n" +
                "    # ------------------------------------------------\n" +
                "\n" +
                "    # Find contours\n" +
                "    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)\n" +
                "    \n" +
                "    MIN_AREA = 0\n" +
                "    valid = [c for c in contours if cv2.contourArea(c) > MIN_AREA]\n" +
                "    valid.sort(key=cv2.contourArea, reverse=True)\n" +
                "    largestContour = valid[0] if valid else np.array([])\n" +
                "    cv2.drawContours(image, [largestContour], -1, (255,255,100), 2)\n" +
                "    #(x,y), r = cv2.minEnclosingCircle(largestContour)\n" +
                "    #center = (int (x), int (y))\n" +
                "    #radius = int (r)\n" +
                "    #cv2.circle(image, center, radius, (255,0,100),2)\n" +
                "\n" +
                "    M = cv2.moments(largestContour)\n" +
                "    if M[\"m00\"] == 0:\n" +
                "        cx=-1\n" +
                "        cy=-1\n" +
                "    else:\n" +
                "        cx = int(M[\"m10\"] / M[\"m00\"])\n" +
                "        cy = int(M[\"m01\"] / M[\"m00\"])\n" +
                "    #largestContour=np.array([])\n" +
                "    #edges = cv2.Canny(mask, 50, 150)\n" +
                "\n" +
                "    #circles = cv2.HoughCircles(edges, cv2.HOUGH_GRADIENT, \n" +
                "    #dp=1.2, minDist=25, param1=100, param2=25, minRadius=10, maxRadius=50)\n" +
                "    #if circles is not None:\n" +
                "    #    circles = np.uint16(np.around(circles))\n" +
                "    #    for i in circles[0,:]:\n" +
                "    #        cv2.circle(image, (i[0], i[1]), i[2], (0,255,0),2)\n" +
                "\n" +
                "    cv2.putText(image, f\"Center: ({cx}, {cy})\", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)\n" +
                "    cv2.circle(image, (cx,cy), 1, (0,0,255), -1)\n" +
                "    image = green_mask\n" +
                "    #cv2.rectangle(image, (0,0), (640,200), (0,0,0), -1)\n" +
                "    \n" +
                "    return largestContour, image, llpython";
//        limelight.uploadPython(pythonCode, 0);
//        limelight.pipelineSwitch(0);
        limelight.start();

        waitForStart();

        while (opModeIsActive()){
            LLResult result = limelight.getLatestResult();
            double tx = result.getTx();
            double ty = result.getTy();
            double x = 5.25*Math.sin(Math.toRadians(tx))/Math.tan(Math.toRadians(ty));
            double y = -5.25*Math.cos(Math.toRadians(tx))/Math.tan(Math.toRadians(ty));
            telemetry.addData("tx", tx);
            telemetry.addData("ty", ty);

            telemetry.addData("x", x);
            telemetry.addData("y", y);

            telemetry.update();
        }
    }
}
