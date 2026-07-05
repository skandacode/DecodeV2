package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.utils.CachedMotor;

@Configurable
public class Intake {
    CachedMotor front;
    CachedMotor transfer;

    TouchSensor beamBreakInside;
    TouchSensor beamBreakOutside;
    RevColorSensorV3 intakeSensor;

    private double cachedDistance;
    public static double intakeSensorThresh = 3.3;
    public static double currentLimit = 4;
    public double intakePower;

    public Intake(HardwareMap hardwareMap) {
        front = new CachedMotor(hardwareMap, "frontIntake");
        transfer = new CachedMotor(hardwareMap, "transferIntake");

        intakeSensor = hardwareMap.get(RevColorSensorV3.class, "goodIntakeSensor");

        beamBreakInside = hardwareMap.touchSensor.get("break1");
        beamBreakOutside = hardwareMap.touchSensor.get("break2");

        front.setCurrentAlert(4.0);
        transfer.setCurrentAlert(4.0);

        update();
    }

    public void setPower(double power) {
        intakePower = power;
        setIntakePower(power);
        setTransferPower(power);
    }

    public void setIntakePower(double power){
        if (front.isOverCurrent()){
            double current = front.getCurrentDraw();
            if (current != 0) {
                power = power * currentLimit / current;
            }
        }
        front.set(power);
    }

    public void setTransferPower(double power){
        transfer.set(power);
    }
    public boolean getDetected(){
        double distance = getDistance();
        System.out.println("good "+distance);
        return distance< intakeSensorThresh;
    }

    public double getDistance(){
        cachedDistance = intakeSensor.getDistance(DistanceUnit.CM);
        return cachedDistance;
    }

    public void update() {
        front.update();
        transfer.update();
    }
    public boolean getBeamBreakInside(){
        return beamBreakInside.isPressed();
    }
    public boolean getBeamBreakOutside(){
        return beamBreakOutside.isPressed();
    }
}
