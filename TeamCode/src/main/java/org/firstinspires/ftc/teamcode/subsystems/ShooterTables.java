package org.firstinspires.ftc.teamcode.subsystems;


public class ShooterTables {
    public static double getHoodPosition(double distance) {
        return (-3.04567e-8) * Math.pow(distance, 4)
                + (1.3578e-5)  * Math.pow(distance, 3)
                - (0.00220059) * Math.pow(distance, 2)
                + (0.1549)* distance
                - 3.44543;
    }
    public static double getShooterVelocity(double distance) {
        return (-3.97447e-5) * Math.pow(distance, 4)
                + (1.79756e-2)  * Math.pow(distance, 3)
                - (2.92533)     * Math.pow(distance, 2)
                + (209.10536)   * distance
                - 4051.96864;
    }
}
