package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class ShooterTables {
    public static double getHoodPosition(double distance) {
        return -1.34068e-8 * Math.pow(distance, 4)
                + 0.00000533003 * Math.pow(distance, 3)
                - 0.000773036   * Math.pow(distance, 2)
                + 0.051215    * distance
                - 0.73333;
    }
    public static double getShooterVelocity(double distance) {
            return -0.00000527297 * Math.pow(distance, 4)
                    + 0.00184391 * Math.pow(distance, 3)
                    -0.227349 * Math.pow(distance, 2)
                    + 19.86808 * distance
                    + 697.3213;

    }

    public static double actualShooterVelocityNoLoad(double targetVelocity) {
        return targetVelocity;
    }

    public static double getHoodAngleChange(double loadedVelocity, double distance){
        //should be negative
        double error = loadedVelocity - actualShooterVelocityNoLoad(getShooterVelocity(distance));
        if (distance < hoodAdjustDistanceThreshold){
            error = 0;
        }
        System.out.println("Error "+ error);
        return error * hoodAngleChangePer100ticksPerSecondError/100;
    }

    public static double hoodAngleChangePer100ticksPerSecondError = 0.04;

    public static double hoodAdjustDistanceThreshold = 110;
}
