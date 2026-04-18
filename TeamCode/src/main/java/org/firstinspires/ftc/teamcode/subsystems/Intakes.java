package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import solverslib.hardware.motors.Motor;

@Configurable
public class Intakes {
    Motor frontIntake;
    Motor transferIntake;

    TouchSensor goodBeamBreakInside;
    TouchSensor goodBeamBreakOutside;

    RevColorSensorV3 goodIntakeSensor;

    private double cachedGoodDistance;

    public static double goodIntakeSensorThresh = 3.2;


    public double goodPower, badPower;

    public static double frontIntakeCurrentLimit = 4;

    public Intakes(HardwareMap hardwareMap) {
        frontIntake = new Motor(hardwareMap, "frontIntake");
        transferIntake = new Motor(hardwareMap, "transferIntake");

        goodIntakeSensor = hardwareMap.get(RevColorSensorV3.class, "goodIntakeSensor");

        goodBeamBreakInside = hardwareMap.touchSensor.get("break1");
        goodBeamBreakOutside = hardwareMap.touchSensor.get("break1");

        frontIntake.setCurrentAlert(4.0);

        update();
    }

    public void setGoodIntakePower(double power) {
        goodPower = power;
        setFrontIntakePower(power);
        setTransferIntakePower(power);
    }

    public void setFrontIntakePower(double power){
        if (frontIntake.isOverCurrent()){
            double current = frontIntake.getCurrentDraw();
            if (current != 0) {
                power = power * frontIntakeCurrentLimit / current;
            }
        }
        frontIntake.set(power);
    }

    public void setTransferIntakePower(double power){
        transferIntake.set(power);
    }

    public void setBadIntakePower(double power) {}
    public boolean getGoodIntakeDetected(){
        cachedGoodDistance = goodIntakeSensor.getDistance(DistanceUnit.CM);
        double distance = getGoodIntakeDistance();
        System.out.println("good "+distance);
        return distance<goodIntakeSensorThresh;
    }

    public double getGoodIntakeDistance(){
        return cachedGoodDistance;
    }

    public void update() {
        frontIntake.update();
        transferIntake.update();

    }
    public boolean getGoodBeamBreakInside(){
        return goodBeamBreakInside.isPressed();
    }
    public boolean getGoodBeamBreakOutside(){
        return goodBeamBreakOutside.isPressed();
    }
}
