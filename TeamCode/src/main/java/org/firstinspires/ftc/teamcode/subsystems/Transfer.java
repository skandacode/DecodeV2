package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.CachedServo;

@Configurable

public class Transfer {
    CachedServo lowerGate;
    CachedServo kicker;


    public static double kickerKickPos = 0.36;
    public static double kickerIdlePos = 0.31;


    public static double lowerGateOpen = 0.55;
    public static double lowerGateClosed = 0.79;

    public boolean kicked = false;

    public Transfer(HardwareMap hardwareMap) {
        lowerGate = new CachedServo(hardwareMap, "lowerGate");
        kicker = new CachedServo(hardwareMap, "kicker");
    }

    public void update() {
        lowerGate.update();
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

    public void setLowerGate(boolean open){
        if (open){
            lowerGate.setPosition(lowerGateOpen);
        } else {
            lowerGate.setPosition(lowerGateClosed);
        }
    }

}
