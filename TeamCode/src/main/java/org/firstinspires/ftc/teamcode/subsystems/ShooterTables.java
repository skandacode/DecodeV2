package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class ShooterTables {

    public static double getHoodPosition(double distance) {
        return -(4.27737e-9) * Math.pow(distance, 4)
                + 0.00000188378 * Math.pow(distance, 3)
                - 0.000311533 * Math.pow(distance, 2)
                + 0.0249263 * distance
                - 0.12009;
    }
    public static double getShooterVelocity(double distance) {
        double increase = 0;
        if (distance>120){
            increase = 0;
        }
        return 4.68162e-9 * Math.pow(distance, 6)
                - 0.00000309277 * Math.pow(distance, 5)
                + 0.000797735 * Math.pow(distance, 4)
                - 0.102784 * Math.pow(distance, 3)
                + 6.96598 * Math.pow(distance, 2)
                - 227.03405 * distance
                + 3936.46218 + increase;
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

    public static double hoodAngleChangePer100ticksPerSecondError = 0.00;

    public static double hoodAdjustDistanceThreshold = 10;

    public static double getBalltimeinair(double distance){
        return 0.6;
    }
}
