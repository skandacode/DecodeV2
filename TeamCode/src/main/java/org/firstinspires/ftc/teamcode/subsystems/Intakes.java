package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import solverslib.hardware.motors.Motor;

@Configurable
public class Intakes {
    Motor goodIntakeMotor;
    Motor badIntakeMotor;

    private double goodIntakeCurrent = 0;
    private double badIntakeCurrent = 0;

    public static double goodIntake3Thresh = 5.0;

    public Intakes(HardwareMap hardwareMap) {
        goodIntakeMotor = new Motor(hardwareMap, "goodIntakeMotor");
        badIntakeMotor = new Motor(hardwareMap, "badIntakeMotor");

        badIntakeMotor.setInverted(true);
    }

    public void setGoodIntakePower(double power) {
        goodIntakeMotor.set(power);
    }
    public void setBadIntakePower(double power) {
        badIntakeMotor.set(power);
    }

    public double getGoodIntakeVelocity(){
        return goodIntakeMotor.getVelocity();
    }
    public double getBadIntakeVelocity(){
        return badIntakeMotor.getVelocity();
    }

    public double getGoodIntakeCurrentDraw(){
        return goodIntakeMotor.getCurrentDraw();
    }
    public double getBadIntakeCurrentDraw(){
        return badIntakeMotor.getCurrentDraw();
    }

    public void update() {
        goodIntakeMotor.update();
        badIntakeMotor.update();
    }

    public void updateCurrent(){
        goodIntakeCurrent = getGoodIntakeCurrentDraw();
        badIntakeCurrent = getBadIntakeCurrentDraw();
    }

    public double getGoodIntakeCurrent(){
        return goodIntakeCurrent;
    }
    public double getBadIntakeCurrent(){
        return badIntakeCurrent;
    }

}
