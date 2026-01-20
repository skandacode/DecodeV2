package org.firstinspires.ftc.teamcode.subsystems;


public class ShooterTables {
    public static double getHoodPosition(double distance) {
        return -1.03121e-8 * Math.pow(distance, 4)
                + 0.00000451516 * Math.pow(distance, 3)
                - 0.000725976   * Math.pow(distance, 2)
                + 0.0517744     * distance
                - 0.793225;
    }
    public static double getShooterVelocity(double distance) {
            return -0.0000131929 * Math.pow(distance, 4)
                    + 0.00548226 * Math.pow(distance, 3)
                    - 0.807222 * Math.pow(distance, 2)
                    + 56.34443 * distance
                    - 67.60176;

    }

    public static double actualShooterVelocity(double distance) {
        return -0.0000189653 * Math.pow(distance, 4)
                + 0.00786296 * Math.pow(distance, 3)
                - 1.17049 * Math.pow(distance, 2)
                + 81.6816 * distance
                - 103.137;
    }

    public static double hoodAngleChangePer100ticksPerSecondError = 0.01;
}
