package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class ShooterTables {

    public static double getHoodPosition(double distance) {
        double increasehood = 0;
        if (distance>100){
            increasehood = -0.01;
        }
        if (distance<70){
            increasehood = 0.0;
        }
        return -8.33333e-9 * Math.pow(distance, 4)
                + 0.00000336908 * Math.pow(distance, 3)
                - 0.000502057 * Math.pow(distance, 2)
                + 0.033713 * distance
                - 0.249207 + increasehood;
    }
    public static double getShooterVelocity(double distance) {
        double increase = 0;
        if (distance>110){
            increase = 0;
        }
        else if (distance<100){
            increase = 30;
        }
        return  -0.00000203963 * Math.pow(distance, 4)
                + 0.000743978 * Math.pow(distance, 3)
                - 0.0541084 * Math.pow(distance, 2)
                + 6.0911 * distance
                + 997.52914 + increase;
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

    public static double hoodAngleChangePer100ticksPerSecondError = 0.02;

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
