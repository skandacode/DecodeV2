package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

import solverslib.hardware.ServoEx;

@Configurable
public class Tilt {
    private ServoEx tilt1, tilt2;

    public static double up1 = 0;
    public static double up2 = 1;

    public static double down1 = 1;
    public static double down2 = 1;


    public Tilt(HardwareMap hwMap){
        tilt1 = new ServoEx(hwMap, "tilt1");
        tilt2 = new ServoEx(hwMap, "tilt2");
    }

    public void retract(){
        tilt1.setPosition(up1);
        tilt2.setPosition(up2);
    }

    public void tilt(){
        tilt1.setPosition(down1);
        tilt2.setPosition(down2);
    }

    public void update(){
        tilt1.update();
        tilt2.update();
    }
}
