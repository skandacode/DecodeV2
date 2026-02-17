package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class ShooterTables {

    public static double getHoodPosition(double distance) {
        return -(1.20625e-8) * Math.pow(distance, 4)
                + 0.00000491312 * Math.pow(distance, 3)
                - 0.000708347 * Math.pow(distance, 2)
                + 0.0448109 * distance
                - 0.42124;
    }
    public static double getShooterVelocity(double distance) {
        double increase = 0;
        if (distance>120){
            increase = 0;
        }
        return 0.00000914568 * Math.pow(distance, 4)
                - 0.00357266 * Math.pow(distance, 3)
                + 0.478749 * Math.pow(distance, 2)
                - 17.54123 * distance
                + 1378.74048 + increase;
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

    public static double hoodAngleChangePer100ticksPerSecondError = 0.03;

    public static double hoodAdjustDistanceThreshold = 10;

    public static double getBalltimeinair(double distance){
        return 0.6;
    }
}
