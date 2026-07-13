package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.CachedServo;

@Configurable

public class Kicker {
    CachedServo kicker;


    public static double kickerKickPos = 0.36;
    public static double kickerIdlePos = 0.31;

    public boolean kicked = false;

    public Kicker(HardwareMap hardwareMap) {
        kicker = new CachedServo(hardwareMap, "kicker");
    }

    public void update() {
        kicker.update();
    }

    public void setKicker(boolean kick){
        if (kick){
            kicker.setPosition(kickerKickPos);
        } else {
            kicker.setPosition(kickerIdlePos);
        }
        kicked = kick;
    }
}
