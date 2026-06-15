package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class ShooterTables {
    public static double minVelocity = 1250;
    public static double getHoodPosition(double distance) {
        double increasehood = 0;

        return (-7.75058e-9 * Math.pow(distance, 4))
                + (0.00000299223 * Math.pow(distance, 3))
                - (0.000432069 * Math.pow(distance, 2))
                + (0.0294068 * distance)
                - 0.208765 + increasehood;
    }
    public static double getShooterVelocity(double distance) {
        double increase = -100;
        if (distance>110){
            increase = -100;
        }
        double vel =  7.40741*distance + 1281.48148 + increase;


        return Math.max(minVelocity, vel);
    }
    public static double minVelocityFar= 2000;
    public static double getHoodPositionFar(double distance) {
        return 9.52207e-9 * Math.pow(distance, 4)
                - 0.00000561802 * Math.pow(distance, 3)
                + 0.00121364 * Math.pow(distance, 2)
                - 0.113385 * distance
                + 4.64188;}
    public static double getShooterVelocityFar(double distance) {
        double y = -0.000186209 * Math.pow(distance, 4)
                + 0.0930645 * Math.pow(distance, 3)
                - 17.23856 * Math.pow(distance, 2)
                + 1410.18729 * distance
                - 41074.8541;
        return Math.max(2100.0,y);
    }

    public static double actualShooterVelocityNoLoad(double targetVelocity) {
        return targetVelocity;
    }

    public static double getHoodAngleChange(double loadedVelocity, double distance){
        //should be negative
        double error = loadedVelocity - actualShooterVelocityNoLoad(getShooterVelocity(distance));
        if (distance < 136){
            error = 0;
        }
        System.out.println("Error "+ error);
        double hoodchangeamount = -(1.2971173e-7) * Math.pow(distance, 4)
                + 0.000070630471 * Math.pow(distance, 3)
                - 0.014351497 * Math.pow(distance, 2)
                + 1.2903045 * distance
                - 43.327963;
        return error * hoodchangeamount/100;
    }

    public static double hoodAngleChangePer100ticksPerSecondError = 0.03;

    public static double hoodAdjustDistanceThreshold = 90;

    public static double getBalltimeinair(double distance){
        if (distance>120){
            return 0;
        } else if (distance>100) {
            return 0.5;
        } else if (distance>58) {
            return 0.4;
        }else{
            return 0.5;
        }

    }

    public static double instantShotCompensation = 0.02;
}
