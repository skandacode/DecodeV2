package solverslib.gamepad;

import java.util.function.BooleanSupplier;

public abstract class Button extends Trigger {
    public Button() {
    }

    public Button(BooleanSupplier isPressed) {
        super(isPressed);
    }
}
