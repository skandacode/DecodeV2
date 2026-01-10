/*----------------------------------------------------------------------------*/
/* Copyright (c) 2008-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package solverslib.gamepad;

import java.util.function.BooleanSupplier;

public class Trigger {

    private final BooleanSupplier m_isActive;

    public Trigger(BooleanSupplier isActive) {
        m_isActive = isActive;
    }

    public Trigger() {
        m_isActive = () -> false;
    }

    public boolean get() {
        return m_isActive.getAsBoolean();
    }

    public Trigger and(Trigger trigger) {
        return new Trigger(() -> get() && trigger.get());
    }

    public Trigger or(Trigger trigger) {
        return new Trigger(() -> get() || trigger.get());
    }

    public Trigger negate() {
        return new Trigger(() -> !get());
    }

}
