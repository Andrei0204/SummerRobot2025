package org.firstinspires.ftc.teamcode.robotkick.Modules.misc;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roboDrive.math.Vector;
import org.firstinspires.ftc.teamcode.robotkick.Modules.outtake.Claw;
import org.opencv.core.Mat;

@Config
public class Limelight {

    public Limelight3A limelight;
    boolean pressed = false;
    static double constx = 14.5;
    static double consty = 14.5;
    public static double constxx=12.5;
    public static double constxy=8;//6
    public static double gradevar=Math.PI/6;
    Vector rotatedPose = null;
    public LLResult result=null;
    public Limelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

    }

    public void blue_yllw() {
        limelight.pipelineSwitch(2);
        limelight.uploadPython("import cv2\n" +
                "import numpy as np\n" +
                "\n" +
                "def get_orientation(contour):\n" +
                "            rect = cv2.minAreaRect(contour)            \n" +
                "            # Calculate angle (orientation)\n" +
                "            angle = rect[2]\n" +
                "            if rect[1][0] < rect[1][1]:\n" +
                "                angle = 90 + angle\n" +
                "\n" +
                "def runPipeline(image, llrobot):\n" +
                "    # Initialize variables\n" +
                "    height, width = image.shape[:2]\n" +
                "    area = height * width\n" +
                "    min_area = 0.02 * area\n" +
                "    max_area = 0.45 * area\n" +
                "    distance_threshold = 170\n" +
                "    \n" +
                "    # Blur the image to reduce noise\n" +
                "    blurred = cv2.GaussianBlur(image, (5, 5), 0)\n" +
                "    \n" +
                "    # Convert to HSV color space\n" +
                "    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)\n" +
                "    \n" +
                "    # Define color ranges\n" +
                "    yellow_range = ((30, 110, 110), (60, 255, 255))\n" +
                "    blue_range = ((100, 100, 80), (140, 255, 255))\n" +
                "    red_range1 = ((0, 100, 100), (10, 255, 255))\n" +
                "    red_range2 = ((160, 100, 100), (180, 255, 255))\n" +
                "    \n" +
                "    # Create masks\n" +
                "    yellow_mask = cv2.inRange(hsv, yellow_range[0], yellow_range[1])\n" +
                "    blue_mask = cv2.inRange(hsv, blue_range[0], blue_range[1])\n" +
                "    red_mask1 = cv2.inRange(hsv, red_range1[0], red_range1[1])\n" +
                "    red_mask2 = cv2.inRange(hsv, red_range2[0], red_range2[1])\n" +
                "    red_mask = cv2.bitwise_or(red_mask1, red_mask2)\n" +
                "    \n" +
                "    # Combine masks\n" +
                "    combined_mask = cv2.bitwise_or(yellow_mask, cv2.bitwise_or(blue_mask, red_mask))\n" +
                "    \n" +
                "    # Find contours\n" +
                "    contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)\n" +
                "    \n" +
                "    # Filter contours by size and calculate centers\n" +
                "    valid_contours = []\n" +
                "    centers = []\n" +
                "    for contour in contours:\n" +
                "        area = cv2.contourArea(contour)\n" +
                "        if min_area < area < max_area:\n" +
                "            M = cv2.moments(contour)\n" +
                "            if M[\"m00\"] != 0:\n" +
                "                cX = int(M[\"m10\"] / M[\"m00\"])\n" +
                "                cY = int(M[\"m01\"] / M[\"m00\"])\n" +
                "                valid_contours.append((contour, (cX, cY)))\n" +
                "                centers.append((cX, cY))\n" +
                "    \n" +
                "    # Remove contours that are too close\n" +
                "    filtered_contours = []\n" +
                "    for i, (contour, center) in enumerate(valid_contours):\n" +
                "        too_close = False\n" +
                "        for j, other_center in enumerate(centers):\n" +
                "            if i != j and np.linalg.norm(np.array(center) - np.array(other_center)) < distance_threshold:\n" +
                "                too_close = True\n" +
                "                break\n" +
                "        if not too_close:\n" +
                "            filtered_contours.append(contour)\n" +
                "    \n" +
                "    # Process remaining contours\n" +
                "    selected_contour = None\n" +
                "    selected_color = None\n" +
                "    for contour in filtered_contours:\n" +
                "        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)\n" +
                "        cv2.drawContours(mask, [contour], 0, 255, -1)\n" +
                "        mean_color = cv2.mean(hsv, mask=mask)[:3]\n" +
                "        \n" +
                "        if 110 < mean_color[0] < 120:  # Blue\n" +
                "            color = \"Blue\"\n" +
                "        elif 20 < mean_color[0] < 40:  # Yellow\n" +
                "            color = \"Yellow\"\n" +
                "        else:\n" +
                "            continue  # Ignore red\n" +
                "        \n" +
                "        if selected_contour is None or (color == \"Blue\" and selected_color != \"Blue\"):\n" +
                "            selected_contour = contour\n" +
                "            selected_color = color\n" +
                "    \n" +
                "    # Initialize return values\n" +
                "    largest_contour = np.array([[]])\n" +
                "    llpython = [0, 0, 0, 0, 0, 0, 0, 0]\n" +
                "    \n" +
                "    # Process selected contour\n" +
                "    if selected_contour is not None:\n" +
                "        largest_contour = selected_contour\n" +
                "        rect = cv2.minAreaRect(largest_contour)\n" +
                "        angle = rect[2]\n" +
                "        if rect[1][0] < rect[1][1]:\n" +
                "            angle = 90 + angle\n" +
                "        \n" +
                "        # Draw contour and information on image\n" +
                "        cv2.drawContours(image, [selected_contour], 0, (0, 255, 0), 2)\n" +
                "        M = cv2.moments(selected_contour)\n" +
                "        if M[\"m00\"] != 0:\n" +
                "            cX = int(M[\"m10\"] / M[\"m00\"])\n" +
                "            cY = int(M[\"m01\"] / M[\"m00\"])\n" +
                "            cv2.circle(image, (cX, cY), 5, (255, 0, 0), -1)\n" +
                "            cv2.putText(image, f\"{selected_color}: {angle:.2f} deg\", (cX - 20, cY - 20),\n" +
                "                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)\n" +
                "        \n" +
                "        # Populate llpython array\n" +
                "        llpython = [1, cX, cY, angle, 0, 0, 0, 0]\n" +
                "    \n" +
                "    # Add text to image\n" +
                "    cv2.putText(image, \"Limelight Python Script\", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)\n" +
                "    \n" +
                "    return largest_contour, image, llpython", 2);
    }

    public void blue_yllw_nopython() {
        limelight.pipelineSwitch(0);
    }
    public void blue_nopython(){
        limelight.pipelineSwitch(2);
    }
    public void red_nopython(){
        limelight.pipelineSwitch(3);
    }
    public Pose2d getdistance() {

         result = limelight.getLatestResult();
        if (result != null) {
            double ypos = (Math.tan(Math.toRadians(result.getTx())) * consty);
            limelight.start();
            return new Pose2d(
                    (Math.tan(Math.toRadians(result.getTy())))*constx,
                    ypos,
                    0);

        }
        return new Pose2d(0,0,0);
//        double[] Arr = result.getPythonOutput();
  //      rotator.setPosition((0.55 / 180 * Arr[3]) + 0.15);
    }
    public Pose2d    getdistanceVector(double heading) {

        result = limelight.getLatestResult();


        if (result != null) {
            double ypos = (Math.tan(Math.toRadians(result.getTx())) * consty);
            rotatedPose = new Vector((-(Math.tan(Math.toRadians(result.getTy()))) * constx),
                    ypos,
                    0);
            rotatedPose =rotatedPose.rotateBy(rotatedPose,-Math.toRadians(heading));
/*            if(Math.abs(rotatedPose.getY())<3 && Math.abs(rotatedPose.getX())<2)
                rotatedPose= new Vector(0.01,0.01,0);*/
            return new Pose2d(rotatedPose.getX(),rotatedPose.getY(),0);

        }
        return new Pose2d(0,0,0);
//        double[] Arr = result.getPythonOutput();
        //      rotator.setPosition((0.55 / 180 * Arr[3]) + 0.15);
    }
    public Pose2d    getdistanceVectoranlge(double heading) {

        result = limelight.getLatestResult();
        if (result != null) {


            //double ypos = ((constx*Math.tan(gradevar+ Math.toRadians(result.getTx())))-constxy);
            double ypos = (Math.tan(Math.toRadians(result.getTx())) * constxy);
            double xpos = -((constx*Math.tan(gradevar+ Math.toRadians(result.getTy())))-constxx);
            if(result.getTx()==0)
                ypos=0;
            if(result.getTy()==0)
                xpos=0;
            rotatedPose = new Vector(xpos,
                    ypos,
                    0);
            rotatedPose =rotatedPose.rotateBy(rotatedPose,-Math.toRadians(heading));
            return new Pose2d(rotatedPose.getX(),rotatedPose.getY(),0);

        }
        return new Pose2d(0,0,0);
    }
}


