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
        return -1.81818e-7 * Math.pow(distance, 4)
                + 0.000099596 * Math.pow(distance, 3)
                - 0.0202955 * Math.pow(distance, 2)
                + 1.8248 * distance
                - 60.76524;}
    public static double getShooterVelocityFar(double distance) {
        double y = 0.0000606061 * Math.pow(distance, 4)
                - 0.0361616 * Math.pow(distance, 3)
                + 7.99848 * Math.pow(distance, 2)
                - 771.38167 * distance
                + 28859.0476;
        return Math.max(1550,y);
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
        return error * hoodAngleChangePer100ticksPerSecondError/100;
    }
    public static double getHoodAngleChangeFar(double loadedVelocity, double distance) {
        double error = loadedVelocity - getShooterVelocityFar(distance);
        if (distance < 136) {
            error = 0;
        }
        double hoodchangeamount =0;
        return error * hoodchangeamount / 100;
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
