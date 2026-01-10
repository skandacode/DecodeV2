package solverslib.drivebase;

import solverslib.geometry.Vector2d;
import solverslib.hardware.motors.Motor;


public class MecanumDrive extends RobotDrive {
    private double rightSideMultiplier;

    Motor[] motors;

    /**
     * Sets up the constructor for the mecanum drive.
     * Automatically inverts right side by default
     *
     * @param frontLeft  the front left motor
     * @param frontRight the front right motor
     * @param backLeft   the back left motor
     * @param backRight  the back right motor
     */
    public MecanumDrive(Motor frontLeft, Motor frontRight, Motor backLeft, Motor backRight) {
        motors = new Motor[]{frontLeft, frontRight, backLeft, backRight};
        rightSideMultiplier =1;
    }
    /**
     * Sets the range of the input, see RobotDrive for more info.
     *
     * @param min The minimum value of the range.
     * @param max The maximum value of the range.
     */
    public void setRange(double min, double max) {
        super.setRange(min, max);
    }

    /**
     * Stop the motors.
     */
    @Override
    public void stop() {
        for (Motor x : motors) {
            x.stopMotor();
        }
    }

    public void update(){
        for (Motor motor : motors) {
            motor.update();
        }
    }

    /**
     * Drives the robot from the perspective of the robot itself rather than that
     * of the driver.
     *
     * @param strafeSpeed  the horizontal speed of the robot, derived from input
     * @param forwardSpeed the vertical speed of the robot, derived from input
     * @param turnSpeed    the turn speed of the robot, derived from input
     */
    public void driveRobotCentric(double strafeSpeed, double forwardSpeed, double turnSpeed) {
        strafeSpeed = clipRange(strafeSpeed);
        forwardSpeed = clipRange(forwardSpeed);
        turnSpeed = clipRange(turnSpeed);

        Vector2d input = new Vector2d(strafeSpeed, forwardSpeed);

        double theta = input.angle();

        double[] wheelSpeeds = new double[4];
        wheelSpeeds[MotorType.kFrontLeft.value] = Math.sin(theta + Math.PI / 4);
        wheelSpeeds[MotorType.kFrontRight.value] = Math.sin(theta - Math.PI / 4);
        wheelSpeeds[MotorType.kBackLeft.value] = Math.sin(theta - Math.PI / 4);
        wheelSpeeds[MotorType.kBackRight.value] = Math.sin(theta + Math.PI / 4);

        normalize(wheelSpeeds, input.magnitude());

        wheelSpeeds[MotorType.kFrontLeft.value] += turnSpeed;
        wheelSpeeds[MotorType.kFrontRight.value] -= turnSpeed;
        wheelSpeeds[MotorType.kBackLeft.value] += turnSpeed;
        wheelSpeeds[MotorType.kBackRight.value] -= turnSpeed;

        normalize(wheelSpeeds);

        driveWithMotorPowers(
                wheelSpeeds[MotorType.kFrontLeft.value],
                wheelSpeeds[MotorType.kFrontRight.value],
                wheelSpeeds[MotorType.kBackLeft.value],
                wheelSpeeds[MotorType.kBackRight.value]
        );
    }

    /**
     * Drives the motors directly with the specified motor powers.
     *
     * @param frontLeftSpeed    the speed of the front left motor
     * @param frontRightSpeed   the speed of the front right motor
     * @param backLeftSpeed     the speed of the back left motor
     * @param backRightSpeed    the speed of the back right motor
     */
    public void driveWithMotorPowers(double frontLeftSpeed, double frontRightSpeed,
                                     double backLeftSpeed, double backRightSpeed) {
        motors[MotorType.kFrontLeft.value]
                .set(frontLeftSpeed * maxOutput);
        motors[MotorType.kFrontRight.value]
                .set(frontRightSpeed * rightSideMultiplier * maxOutput);
        motors[MotorType.kBackLeft.value]
                .set(backLeftSpeed * maxOutput);
        motors[MotorType.kBackRight.value]
                .set(backRightSpeed * rightSideMultiplier * maxOutput);
    }

}
