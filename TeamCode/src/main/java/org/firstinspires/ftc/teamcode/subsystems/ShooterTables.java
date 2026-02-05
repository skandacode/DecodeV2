package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class ShooterTables {

    public static double getHoodPosition(double distance) {
        return -1.34068e-8 * Math.pow(distance, 4)
                + 0.00000533003 * Math.pow(distance, 3)
                - 0.000773036   * Math.pow(distance, 2)
                + 0.051215    * distance
                - 0.74;
    }
    public static double getShooterVelocity(double distance) {
        double increase = 0;
        if (distance>100){
            increase = 60;
        }
        return -0.00000527297 * Math.pow(distance, 4)
                + 0.00184391 * Math.pow(distance, 3)
                -0.227349 * Math.pow(distance, 2)
                + 19.86808 * distance
                + 680 + increase;
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

    public static double hoodAngleChangePer100ticksPerSecondError = 0.07;

    public static double hoodAdjustDistanceThreshold = 110;

    public static double balltimeinair = 1;
}
