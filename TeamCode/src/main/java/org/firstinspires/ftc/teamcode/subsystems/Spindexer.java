package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

import solverslib.hardware.ServoEx;

@Configurable

public class Spindexer {
    ServoEx spin1, spin2;

    ServoEx lowerGate;
    ServoEx kicker;

    public static double kickerKickPos = 0.5;
    public static double kickerIdlePos = 0.67;

    public static double lowerGateOpen = 0.65;
    public static double lowerGateClosed = 0.5;

    public enum SpindexerPosition {
        Shoot1(0.37),
        Shoot2(0.58),
        Shoot3(0.8),
        Intake1(0.04),
        Intake2(0.27),
        Intake3(0.49);

        public final double position;

        SpindexerPosition(double position) {
            this.position = position;
        }
    }

    public Spindexer(HardwareMap hardwareMap) {
        spin1 = new ServoEx(hardwareMap, "spindexerServo1");
        spin2 = new ServoEx(hardwareMap, "spindexerServo2");

        lowerGate = new ServoEx(hardwareMap, "lowerGate");
        kicker = new ServoEx(hardwareMap, "kicker");
    }

    public void setPosition(double position) {
        spin1.setPosition(position);
        spin2.setPosition(position);
    }

    public void setPosition(SpindexerPosition pos) {
        setPosition(pos.position);
    }

    public void update() {
        spin1.update();
        spin2.update();
        lowerGate.update();
        kicker.update();

    }

    public void setKickerPos(boolean kick){
        if (kick){
            kicker.setPosition(kickerKickPos);
        } else {
            kicker.setPosition(kickerIdlePos);
        }
    }

    public void setLowerGateOpen(boolean open){
        if (open){
            lowerGate.setPosition(lowerGateOpen);
        } else {
            lowerGate.setPosition(lowerGateClosed);
        }
    }
}
