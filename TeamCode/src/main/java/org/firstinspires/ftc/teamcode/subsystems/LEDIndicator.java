package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;

import solverslib.hardware.ServoEx;

public class LEDIndicator {
    private ServoEx internalIndicator;
    public LEDIndicator(HardwareMap hwMap){
        internalIndicator = new ServoEx(hwMap, "indicator");
        //internalIndicator.setPwm(new PwmControl.PwmRange(500, 2500));
        internalIndicator.setPosition(1);
    }
    public void setRed(){
        internalIndicator.setPosition(0.278);
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
