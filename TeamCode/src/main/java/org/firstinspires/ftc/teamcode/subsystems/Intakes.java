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
    Motor goodIntakeMotor;
    Motor badIntakeMotor;
    TouchSensor badBeamBreak;
    TouchSensor goodBeamBreakInside;
    TouchSensor goodBeamBreakOutside;

    RevColorSensorV3 badIntakeSensor;
    RevColorSensorV3 goodIntakeSensor;

    public static double badIntakeSensorThresh = 2;
    public static double goodIntakeSensorThresh = 3.3;
    public static double goodIntakeSensorThreshBottom = 2.6;


    public double goodPower, badPower;


    public Intakes(HardwareMap hardwareMap) {
        goodIntakeMotor = new Motor(hardwareMap, "goodIntakeMotor");
        badIntakeMotor = new Motor(hardwareMap, "badIntakeMotor");
        badIntakeSensor = hardwareMap.get(RevColorSensorV3.class, "badIntakeSensor");
        goodIntakeSensor = hardwareMap.get(RevColorSensorV3.class, "goodIntakeSensor");

        badIntakeMotor.setInverted(true);

        badBeamBreak = hardwareMap.touchSensor.get("badBeamBreak");
        goodBeamBreakInside = hardwareMap.touchSensor.get("goodBeamBreakInside");
        goodBeamBreakOutside = hardwareMap.touchSensor.get("goodBeamBreakOutside");
    }

    public void setGoodIntakePower(double power) {
        goodPower = power;
        goodIntakeMotor.set(power);
    }
    public void setBadIntakePower(double power) {
        badPower = power;
        badIntakeMotor.set(power);
    }

    public double getGoodCurrent(){
        return goodIntakeMotor.getCurrentDraw();
    }

    public double getBadCurrent(){
        return badIntakeMotor.getCurrentDraw();
    }

    public boolean getBadIntakeDetected(){
        return getBadIntakeDistance()<badIntakeSensorThresh;
    }

    public double getBadIntakeDistance(){
        //not detected ~ 2.8
        //nothing at all ~3.2
        //detected ~1
        return badIntakeSensor.getDistance(DistanceUnit.CM);
    }
    public boolean getGoodIntakeDetected(){
        return getGoodIntakeDistance()<goodIntakeSensorThresh && getGoodIntakeDistance()>goodIntakeSensorThreshBottom;
    }

    public double getGoodIntakeDistance(){
        return goodIntakeSensor.getDistance(DistanceUnit.CM);
    }

    public void update() {
        goodIntakeMotor.update();
        badIntakeMotor.update();
    }
    public boolean getBadBeamBreak(){
        return badBeamBreak.isPressed();
    }
    public boolean getGoodBeamBreakInside(){
        return goodBeamBreakInside.isPressed();
    }
    public boolean getGoodBeamBreakOutside(){
        return goodBeamBreakOutside.isPressed();
    }
}
