package org.firstinspires.ftc.teamcode.subsystems;


public class ShooterTables {
    public static double getHoodPosition(double distance) {
        return (-1.65e-8) * Math.pow(distance, 4)
                + (0.00000685734)  * Math.pow(distance, 3)
                - (0.00104471) * Math.pow(distance, 2)
                + (0.0701471)* (distance)
                - 1.16012;
    }
    public static double getShooterVelocity(double distance) {
        return (-0.0000151859) * Math.pow(distance, 4)
                + (0.00633315)  * Math.pow(distance, 3)
                - (0.952019)     * Math.pow(distance, 2)
                + (67.30893)   * (distance)
                - 322.96513;
    }
}
