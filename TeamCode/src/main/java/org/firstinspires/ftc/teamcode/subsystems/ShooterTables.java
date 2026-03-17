package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class ShooterTables {

    public static double getHoodPosition(double distance) {
        double increasehood = 0;
        if (distance>110){
            increasehood = 0.02;
        }
        if (distance<70){
            increasehood = 0.02;
        }
        return 2.1664635e-14*Math.pow(distance,8)
                - 1.6743242e-11*Math.pow(distance,7)
                + 5.5440512e-9*Math.pow(distance,6)
                - 0.0000010256883*Math.pow(distance,5)
                + 0.00011577729*Math.pow(distance,4)
                - 0.0081510486*Math.pow(distance,3)
                + 0.34883167*Math.pow(distance,2)
                - 8.2726411*distance
                + 83.409692+increasehood;
    }
    public static double getShooterVelocity(double distance) {
        double increase = 0;
        if (distance>110){
            increase = 20;
        }
        else if (distance<70){
            increase = 10;
        }
        return 4.9163953e-12*Math.pow(distance,8)
                - 3.7741923e-9*Math.pow(distance,7)
                + 0.0000012447575*Math.pow(distance,6)
                - 0.00023019349*Math.pow(distance,5)
                + 0.02609685*Math.pow(distance,4)
                - 1.8565751*Math.pow(distance,3)
                + 80.853514*Math.pow(distance,2)
                - 1957.78206*distance
                + 21157.554 + increase;
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

    public static double instantShotCompensation = 0.05;
}
