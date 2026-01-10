package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

import solverslib.hardware.motors.Motor;

@Configurable
public class Intakes {
    Motor goodIntakeMotor;
    Motor badIntakeMotor;

    public Intakes(HardwareMap hardwareMap) {
        goodIntakeMotor = new Motor(hardwareMap, "goodIntakeMotor");
        badIntakeMotor = new Motor(hardwareMap, "badIntakeMotor");
    }

    public void setGoodIntakePower(double power) {
        goodIntakeMotor.set(power);
    }
    public void setBadIntakeMotor(double power) {
        badIntakeMotor.set(power);
    }

    public void update() {
        goodIntakeMotor.update();
        badIntakeMotor.update();
    }
}
