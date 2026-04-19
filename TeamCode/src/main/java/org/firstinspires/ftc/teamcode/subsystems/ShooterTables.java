package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class ShooterTables {
    public static double minVelocity = 1250;
    public static double getHoodPosition(double distance) {
        double increasehood = 0;
        if (distance>100){
            increasehood = -0.01;
        }else{
            return 0.5;
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
        double vel =  -0.0000326216 * Math.pow(distance, 4)
                + 0.0120174 * Math.pow(distance, 3)
                - 1.52444 * Math.pow(distance, 2)
                + 85.95417 * distance
                - 556.71684 + increase;

        return Math.max(minVelocity, vel);
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

    public static double hoodAdjustDistanceThreshold = 90;

    public static double getBalltimeinair(double distance){
        if (distance<110){
        return 0.6;}
        else{
            return 0;
        }
    }

    public static double instantShotCompensation = 0.02;
}
