package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;

import solverslib.hardware.ServoEx;

@Configurable
public class Spindexer {
    ServoEx servo1, servo2;

    public enum SpindexerPosition {
        Shoot1(0.2),
        Shoot2(0.4),
        Shoot3(0.6),
        Intake1(0.8),
        Intake2(1.0),
        Intake3(1.0);

        public final double position;

        SpindexerPosition(double position) {
            this.position = position;
        }
    }

    public Spindexer(HardwareMap hardwareMap) {
        servo1 = new ServoEx(hardwareMap, "spindexerServo1");
        servo2 = new ServoEx(hardwareMap, "spindexerServo2");
    }

    public void setPosition(double position) {
        servo1.setPosition(position);
        servo2.setPosition(position);
    }

    public void setPosition(SpindexerPosition pos) {
        setPosition(pos.position);
    }

    public void update() {
        servo1.update();
        servo2.update();
    }
}
