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
        double increase = 0;
        if (distance>110){
            increase = 0;
        }
        else if (distance<100){
            increase = 0;
        }
        double vel = (-0.0000101981 * Math.pow(distance, 4))
                + (0.00430653 * Math.pow(distance, 3))
                - (0.614714 * Math.pow(distance, 2))
                + (43.2162 * distance)
                + 147.83217 + increase;

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
        if (distance>120){
            return 0.7;
        } else if (distance>100) {
            return 0.4;
        } else if (distance>58) {
            return 0.3;
        }else{
            return 0.5;
        }


    }

    public static double instantShotCompensation = 0.02;
}
