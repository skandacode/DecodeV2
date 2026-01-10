package org.firstinspires.ftc.teamcode.subsystems;


public class ShooterTables {
    public static double getHoodPosition(double distance){
        return (1.71189e-12) * Math.pow(distance, 4)
                + (2.18435e-7)  * Math.pow(distance, 3)
                - 0.000096597   * Math.pow(distance, 2)
                + 0.0146982     * distance
                + 0.0972414;    }
    public static double getShooterVelocity(double distance){
        return -0.0000132801 * Math.pow(distance, 4)
                + 0.00581041   * Math.pow(distance, 3)
                - 0.919645     * Math.pow(distance, 2)
                + 66.7719      * distance
                - 690.84713;}
}
