package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class ShooterTables {

    public static double getHoodPosition(double distance) {
        double increasehood = 0;
        if (distance>100){
            increasehood = -0.2;
        }
        if (distance<70){
            increasehood = 0.0;
        }
        return 0.00000782994 * Math.pow(distance, 2)
                + 0.00176365 * distance
                + 0.396957+increasehood;
    }
    public static double getShooterVelocity(double distance) {
        double increase = 0;
        if (distance>110){
            increase = 00;
        }
        else if (distance<100){
            increase = -10;
        }
        return -0.00000301537 * Math.pow(distance, 4)
                + 0.000969238 * Math.pow(distance, 3)
                - 0.111927 * Math.pow(distance, 2)
                + 12.63867 * distance
                + 927.88974 + increase;
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
        if (distance<110){
        return 0.6;}
        else{
            return 0;
        }
    }

    public static double instantShotCompensation = 0.02;
}
