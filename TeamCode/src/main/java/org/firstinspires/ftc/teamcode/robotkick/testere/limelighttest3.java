package org.firstinspires.ftc.teamcode.robotkick.testere;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roboDrive.math.Vector;

@Config
@TeleOp
public class limelighttest3 extends LinearOpMode {

    private Limelight3A limelight;

    double mate;
    double ticksperdegree =0.003;
    Servo test;
    @Override
    public void runOpMode() throws InterruptedException {
        test =hardwareMap.get(Servo.class,"Rotator");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.uploadPython("import cv2\n" +
                "import numpy as np\n" +
                "\n" +
                "def drawDecorations(image, angle, center, box):\n" +
                "    # Draw the rectangle\n" +
                "    cv2.drawContours(image, [np.int0(box)], 0, (0, 255, 0), 2)\n" +
                "    \n" +
                "    # Draw the center point\n" +
                "    cv2.circle(image, tuple(map(int, center)), 5, (0, 0, 255), -1)\n" +
                "    \n" +
                "    # Draw the angle\n" +
                "    cv2.putText(image, f\"Angle: {angle:.2f}\", (10, 30), \n" +
                "                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)\n" +
                "\n" +
                "def runPipeline(image, llrobot):\n" +
                "    # Initialize variables\n" +
                "    largestContour = np.array([[]])\n" +
                "    llpython = [0, 0, 0, 0, 0, 0, 0, 0]\n" +
                "    angle = 0\n" +
                "    \n" +
                "    # Convert BGR to HSV\n" +
                "    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)\n" +
                "    \n" +
                "    # Define range of yellow color in HSV\n" +
                "    lower_yellow = np.array([20, 100, 100])\n" +
                "    upper_yellow = np.array([30, 255, 255])\n" +
                "    \n" +
                "    # Threshold the HSV image to get only yellow colors\n" +
                "    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)\n" +
                "    \n" +
                "    # Apply a slight blur to reduce noise\n" +
                "    mask = cv2.GaussianBlur(mask, (5, 5), 0)\n" +
                "    \n" +
                "    # Find contours\n" +
                "    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)\n" +
                "    \n" +
                "    # Calculate the minimum area threshold (10% of the image)\n" +
                "    min_area = image.shape[0] * image.shape[1] * 0.1\n" +
                "    \n" +
                "    valid_contours = []\n" +
                "    for contour in contours:\n" +
                "        area = cv2.contourArea(contour)\n" +
                "        if area >= min_area:\n" +
                "            # Check if the contour is approximately rectangular\n" +
                "            peri = cv2.arcLength(contour, True)\n" +
                "            approx = cv2.approxPolyDP(contour, 0.04 * peri, True)\n" +
                "            if len(approx) == 4:\n" +
                "                valid_contours.append(contour)\n" +
                "    \n" +
                "    if valid_contours:\n" +
                "        # Get the largest valid contour\n" +
                "        largestContour = max(valid_contours, key=cv2.contourArea)\n" +
                "        \n" +
                "        # Find the minimum area rectangle\n" +
                "        rect = cv2.minAreaRect(largestContour)\n" +
                "        box = cv2.boxPoints(rect)\n" +
                "        \n" +
                "        # Calculate angle\n" +
                "        width = rect[1][0]\n" +
                "        height = rect[1][1]\n" +
                "        angle = rect[2]\n" +
                "        \n" +
                "        # Adjust angle to always reference the longest side\n" +
                "        if width < height:\n" +
                "            angle += 90\n" +
                "        \n" +
                "        # Ensure angle is between 0 and 180\n" +
                "        angle %= 180\n" +
                "        \n" +
                "        # Draw decorations\n" +
                "        drawDecorations(image, angle, rect[0], box)\n" +
                "        \n" +
                "        # Populate llpython with results\n" +
                "        llpython = [1, rect[0][0], rect[0][1], width, height, angle, 0, 0]\n" +
                "    \n" +
                "    return largestContour, image, llpython",1);
        limelight.start();
        Vector rotatedPose = null;
        while (opModeInInit()) {

        }
        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            double[] Arr = result.getPythonOutput();
            if(gamepad1.a)
                test.setPosition((ticksperdegree * Arr[5]) + 0.15);
            telemetry.addData("s",Arr[5]);
        }

    }
}


