package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;

import solverslib.hardware.ServoEx;

@Configurable
public class LEDIndicator {
    private ServoEx internalIndicator;
    public static double redColor = 0.29;
    public static double orangeColor = 0.333;
    public static double yellowColor = 0.388;
    public static double greenColor = 0.6;

    public LEDIndicator(HardwareMap hwMap){
        internalIndicator = new ServoEx(hwMap, "indicator");
        //internalIndicator.setPwm(new PwmControl.PwmRange(500, 2500));
        internalIndicator.setPosition(1);
    }
    public void setRed(){
        internalIndicator.setPosition(0.29);
    }
    public void setOrange(){
        internalIndicator.setPosition(0.333);
    }
    public void setYellow(){
        internalIndicator.setPosition(0.388);
    }
    public void setGreen(){
        internalIndicator.setPosition(0.5);
    }
    public void update(){
        internalIndicator.update();
    }
}
