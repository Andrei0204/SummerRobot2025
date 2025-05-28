package org.firstinspires.ftc.teamcode.roboDrive.Wrappers;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.roboDrive.SexyLocalizer;
import org.firstinspires.ftc.teamcode.newEminescu.modules.drive.SexyLocalizerV2;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roboDrive.math.Angles;

public class CoolIMU {

    private final Object imuLock = new Object();
    public IMU imu=null;
    public static double imuAngle = 0;
    public static double imuVelocity = 0;
    public static double imuOffset = 0;

    public CoolIMU(HardwareMap hardwareMap) {

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR));

        imu.initialize(parameters);
        imu.resetYaw();
    }

    public CoolIMU(){

    }

    public void startIMUThread(LinearOpMode opMode, SexyLocalizer localizer) {
        new Thread(() -> {
            imu.resetYaw();
            while ((opMode.opModeIsActive() && !opMode.isStopRequested()) || (opMode.opModeInInit() && !opMode.isStopRequested())) {
                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                synchronized (imuLock) {
                    imuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                    localizer.update();
//                    imuVelocity = imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
                }
            }
        }).start();
    }

    public void startIMUThread(LinearOpMode opMode, SexyLocalizerV2 localizer) {
        new Thread(() -> {
            imu.resetYaw();
            while ((opMode.opModeIsActive() && !opMode.isStopRequested()) || (opMode.opModeInInit() && !opMode.isStopRequested())) {
                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                synchronized (imuLock) {
                    imuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                    localizer.update();
//                    imuVelocity = imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
                }
            }
        }).start();
    }

    public double getAngle()
    {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }


    public void startIMUThread(LinearOpMode opMode) {
        new Thread(() -> {
            imu.resetYaw();
            while ((opMode.opModeIsActive() && !opMode.isStopRequested()) || (opMode.opModeInInit() && !opMode.isStopRequested())) {
                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                synchronized (imuLock) {
                    imuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//                    imuVelocity = imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
                }
            }
        }).start();
    }
    public double getImuAngle(){return imuAngle;}

    public synchronized void resetImu() {
//        imu.resetDeviceConfigurationForOpMode();
//        imuOffset = 0;
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT
//        ));
//        imu.initialize(parameters);
        imu.resetYaw();
    }

    public double getHeading() {
        return Angles.normalize(imuAngle - imuOffset);
    }

    public double getVelocity() {
        return imuVelocity;
    }

}