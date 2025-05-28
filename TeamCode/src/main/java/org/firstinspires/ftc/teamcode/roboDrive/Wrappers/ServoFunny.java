package org.firstinspires.ftc.teamcode.roboDrive.Wrappers;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class ServoFunny {

    public ServoImplEx servo;
    private double position = -1;

    final Object lock = new Object();

    public ServoFunny(ServoImplEx servo){
        this.servo = servo;
    }


    public void setPositionAsync(double position){
        synchronized (lock) {
            this.position = position;
        }
    }

    public void updatePositionAsync(){
        synchronized (lock) {
            if(position != -1)
                servo.setPosition(position);
        }
    }
}
