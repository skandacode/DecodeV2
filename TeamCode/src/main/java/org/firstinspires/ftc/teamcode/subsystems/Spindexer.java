package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

import solverslib.hardware.ServoEx;

@Configurable

public class Spindexer {
    ServoEx spin1, spin2;

    ServoEx lowerGate;
    ServoEx kicker;

    public static double kickerKickPos = 0.45;
    public static double kickerIdlePos = 0.71;

    public static double lowerGateOpen = 0.825;
    public static double lowerGateClosed = 0.7;

    public boolean is_kick = false;

    public enum SpindexerPosition {
        Shoot0(0.17),
        Shoot1(0.38),
        Shoot2(0.59),
        Shoot3(0.81),
        Shoot4(1.0),
        Intake1(0.05),
        Intake2(0.28),
        Intake3(0.5),
        Intake4(0.72);


        public final double position;

        SpindexerPosition(double position) {
            this.position = position;
        }
    }

    private SpindexerPosition currentPosition = SpindexerPosition.Shoot1;

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
        currentPosition = pos;
    }
    public SpindexerPosition getCurrentPosition() {
        return currentPosition;
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
        is_kick = kick;
    }

    public void setLowerGateOpen(boolean open){
        if (open){
            lowerGate.setPosition(lowerGateOpen);
        } else {
            lowerGate.setPosition(lowerGateClosed);
        }
    }
}
