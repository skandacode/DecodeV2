package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class ShooterTables {
    public static double minVelocity = 1000;
    public static double getHoodPosition(double distance) {
        double increasehood = 0;
        return 23.57345 * Math.pow(distance, -0.957616) +increasehood;

    }
    public static double getShooterVelocity(double distance) {
        double increase = 0;
        double vel =  0.0000672119 * Math.pow(distance, 4)
                - 0.0217524 * Math.pow(distance, 3)
                + 2.5497 * Math.pow(distance, 2)
                - 122.58737 * distance
                + 3074.50544 +increase;
        return Math.max(minVelocity, vel);
    }

    public static double getHoodPositionFar(double distance) {
        double y=  6.99074e-7 * Math.pow(distance, 4)
                - 0.000384853 * Math.pow(distance, 3)
                + 0.0789672 * Math.pow(distance, 2)
                - 7.15944 * distance
                + 242.54567;
        return Math.max(y,0.38);}
    public static double getShooterVelocityFar(double distance) {
        double y = - 0.00031736 * Math.pow(distance, 4)
                + 0.175946 * Math.pow(distance, 3)
                - 36.40257 * Math.pow(distance, 2)
                + 3338.84237 * distance
                - 113030.791;
        return Math.max(1450,y);
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

    public static double hoodAdjustDistanceThreshold = 97;

    public static double getBalltimeinair(double distance){
        double y = 9.79737e-8 * Math.pow(distance, 4)
                - 0.0000267513 * Math.pow(distance, 3)
                + 0.00273554 * Math.pow(distance, 2)
                - 0.122443 * distance
                + 2.54104;
        return Math.min(0.7,y);
    }
    public static double getBalltimeinairFar(double distance){
        return 0;
    }

    public static double instantShotCompensation = 0.1;
}
