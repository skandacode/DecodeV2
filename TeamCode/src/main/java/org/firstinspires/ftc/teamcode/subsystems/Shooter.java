package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.utils.PIDFController;
import org.firstinspires.ftc.teamcode.utils.SimpleMotorFeedforward;
import org.firstinspires.ftc.teamcode.utils.CachedServo;
import org.firstinspires.ftc.teamcode.utils.CachedMotor;

@Configurable
public class Shooter {

    private CachedMotor shooterCachedMotor1, shooterCachedMotor2, shooterEncoder1, shooterEncoder2;
    private CachedServo turret1, turret2, hood;

    private CachedServo upperGate;

    VoltageSensor voltageSensor;

    private PIDFController pidf;
    private SimpleMotorFeedforward feedforward;

    private double targetVelocity = 0.0;
    private double currentVelocity = 0.0;

    // --- Flywheel PIDF coefficients ---
    public static double kP = 0.01;
    public static double kI = 0;
    public static double kD = 0;

    public static double kS = 0.11; // Static feedforward
    public static double kV = 0.000387; // Velocity feedforward

    public static boolean enablePIDF = true;

    // --- Turret bounds ---
    public static double turretUpperBound = 0.97;
    public static double turretLowerBound = 0.03;

    // --- Hood bounds ---

    public static double hoodLowerBound = 0.48;
    public static double hoodUpperBound = 0.85;

    // --- Low-pass filter coefficient (for smoothing) ---
    public static double ALPHA = 0.3;
    private double smoothedVelocity = 0.0;

    public static Pose RedGoalPose = new Pose(-70, 62);
    public static Pose BlueGoalPose
            = new Pose(-67, -62);

    public enum Goal{
        RED (RedGoalPose),
        BLUE (BlueGoalPose);

        Pose position;
        Goal(Pose pose) {
            position = pose;
        }
    }

    public static double powerOffset = 0;
    public static double turretOffset = 0;
    public static double limelightOffset = 0;


    public static double upperGateOpenPos = 0.68;
    public static double upperGateClosedPos = 0.56;

    private double prevX, prevY;
    private long prevPosTime;

    // add vx and vy fields
    private double vx = 0.0;
    private double vy = 0.0;

    // angular velocity field
    private double omega = 0.0;
    private double prevHeading = 0.0;
    private long prevHeadingTime = 0;

    // acceleration fields
    private double ax = 0.0;
    private double ay = 0.0;
    private double prevVx = 0.0;
    private double prevVy = 0.0;
    private long prevVelTime = 0;

    private double prevTargetVelocity = 0.0;
    private long prevTargetTime = 0;

    public boolean canReachPos = true;

    public Shooter(HardwareMap hardwareMap) {
        shooterCachedMotor1 = new CachedMotor(hardwareMap, "shooterMotor1");
        shooterCachedMotor2 = new CachedMotor(hardwareMap, "shooterMotor2");

        shooterEncoder1 = new CachedMotor(hardwareMap, "frontright");
        shooterEncoder2 = new CachedMotor(hardwareMap, "shooterMotor2");


        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        upperGate = new CachedServo(hardwareMap, "upperGate");

        turret1 = new CachedServo(hardwareMap, "turret1");
        turret2 = new CachedServo(hardwareMap, "turret2");

        hood = new CachedServo(hardwareMap, "hood");

        pidf = new PIDFController(kP, kI, kD, 0);
        feedforward = new SimpleMotorFeedforward(kS, kV);

        // initialize previous pos time to avoid large dt on first call
        prevPosTime = System.nanoTime();
        prevX = 0.0;
        prevY = 0.0;
        prevVelTime = System.nanoTime();
        prevVx = 0.0;
        prevVy = 0.0;
        prevHeadingTime = System.nanoTime();
        prevHeading = 0.0;

        prevTargetTime = System.nanoTime();
    }

    public void setUpperGate(boolean open){
        if (open){
            upperGate.setPosition(upperGateOpenPos);
        } else {
            upperGate.setPosition(upperGateClosedPos);
        }
    }

    public double[] getAngleDistance(Pose currPosition, Goal target){
        return getAngleDistance(currPosition, target.position);
    }

    public double[] getAngleDistance(Pose currPosition, Pose target){
        double dx = target.getX()-currPosition.getX();
        double dy = target.getY()-currPosition.getY();
        double angle = Math.atan2(dy, dx);
        double turretAngle = Math.toDegrees(-angle + currPosition.getHeading());

        while (Math.abs(turretAngle)>180){
            if (turretAngle>0){
                turretAngle -= 360;
            }else{
                turretAngle +=360;
            }
        }

        double distance = Math.hypot(dx, dy);

        return new double[]{turretAngle, distance};
    }

    public void setTurretPos(double pos){
        double safePos = Range.clip(pos, turretLowerBound, turretUpperBound);
        canReachPos = safePos == pos;
        turret1.setPosition(safePos);
        turret2.setPosition(safePos);

    }

    public double convertDegreestoServoPos(double deg){
        return deg*-0.003222222222222222+0.5;
    }

    public void aimAtTarget(Pose currPosition, Goal target){
        aimAtTarget(currPosition, target.position);
    }
    public void aimAtTargetFar(Pose currPosition, Goal target){
        aimAtTargetFar(currPosition, target.position);
    }
    public void aimAtTargetFar(Pose currPosition, Pose target) {
        double[] angleDistance = getAngleDistance(currPosition, target);
        double angle = angleDistance[0];
        double distance = angleDistance[1];

        double servoPos = convertDegreestoServoPos(angle + turretOffset + limelightOffset);
        servoPos = Range.clip(servoPos, turretLowerBound, turretUpperBound);

        double currVelo = getCurrentVelocity();

        setTurretPos(servoPos);
        setTargetVelocity(Tables.getShooterVelocityFar(distance) + powerOffset);
        setHood(Tables.getHoodPositionFar(distance) + Tables.getHoodAngleChangeFar(currVelo, distance));
    }
    public void aimAtTarget(Pose currPosition, Pose target){
        long currTime = System.nanoTime();
        double dt = (currTime - prevPosTime) / 1e9; // convert ns to seconds

        double computedVx = 0.0;
        double computedVy = 0.0;
        if (prevPosTime != 0 && dt > 1e-6) {
            computedVx = (currPosition.getX() - prevX) / dt;
            computedVy = (currPosition.getY() - prevY) / dt;
        }

        // calculate acceleration the same way velocity is calculated
        double dtVel = (currTime - prevVelTime) / 1e9;
        double computedAx = 0.0;
        double computedAy = 0.0;
        if (prevVelTime != 0 && dtVel > 1e-6) {
            computedAx = (computedVx - prevVx) / dtVel;
            computedAy = (computedVy - prevVy) / dtVel;
        }

        // store to instance fields for external access if needed
        this.vx = computedVx;
        this.vy = computedVy;
        this.ax = computedAx;
        this.ay = computedAy;

        // calculate angular velocity of heading the same way linear velocity is calculated
        double dtHeading = (currTime - prevHeadingTime) / 1e9;
        double computedOmega = 0.0;
        if (prevHeadingTime != 0 && dtHeading > 1e-6) {
            double dHeading = currPosition.getHeading() - prevHeading;
            // wrap to [-π, π] to handle crossing ±π boundary
            while (dHeading > Math.PI) dHeading -= 2 * Math.PI;
            while (dHeading < -Math.PI) dHeading += 2 * Math.PI;
            computedOmega = dHeading / dtHeading;
        }
        this.omega = computedOmega;

        double precomputedDistance = getAngleDistance(currPosition, target)[1];
        double tFlight = Tables.getBalltimeinair(precomputedDistance);
        double tDelay = Tables.instantShotCompensation;

        // The ball inherits the robot's velocity at the moment it launches, NOT
        // at the moment we compute the aim. During the mechanical delay (tDelay)
        // the robot accelerates, so the launch velocity is:
        //   v_launch = v_now + a * tDelay
        //
        // Two sources of positional offset:
        // 1) Robot physically moves during tDelay:  v_now*tDelay + 0.5*a*tDelay²
        // 2) Ball drifts during tFlight at the launch velocity:  v_launch * tFlight
        //
        // Total offset = v_now*(tDelay + tFlight) + a*tDelay*(0.5*tDelay + tFlight)
        double offsetX = this.vx * (tDelay + tFlight) + this.ax * tDelay * (0.5 * tDelay + tFlight);
        double offsetY = this.vy * (tDelay + tFlight) + this.ay * tDelay * (0.5 * tDelay + tFlight);

        Pose realTarget = new Pose(
                target.getX() - offsetX,
                target.getY() - offsetY,
                target.getHeading());


        double[] angleDistance = getAngleDistance(currPosition, realTarget);
        double angle = angleDistance[0];
        double distance = angleDistance[1];

        // Compensate turret angle for the robot's rotation during the mechanical
        // delay only — once the ball leaves, the robot's rotation no longer matters.
        double omegaCompensationDeg = Math.toDegrees(this.omega * tDelay);
        double servoPos = convertDegreestoServoPos(angle + turretOffset + omegaCompensationDeg + limelightOffset);

        double currVelo = getCurrentVelocity();

        servoPos = Range.clip(servoPos, turretLowerBound, turretUpperBound);


        setTurretPos(servoPos);
        setTargetVelocity(Tables.getShooterVelocity(distance) + powerOffset);
        setHood(Tables.getHoodPosition(distance) + Tables.getHoodAngleChange(currVelo, distance));

        // update previous position/time for next velocity calculation
        prevX = currPosition.getX();
        prevY = currPosition.getY();
        prevPosTime = currTime;
        // update previous velocity/time for next acceleration calculation
        prevVx = this.vx;
        prevVy = this.vy;
        prevVelTime = currTime;
        // update previous heading/time for next angular velocity calculation
        prevHeading = currPosition.getHeading();
        prevHeadingTime = currTime;
    }

    public void setTargetVelocity(double target) {
        targetVelocity = target;
        pidf.reset();
    }

    public double getCurrentVelocity() {
        return smoothedVelocity;
    }

    public void setDirectPower(double power) {
        power = power * 12/voltageSensor.getVoltage();
        shooterCachedMotor1.set(power);
        shooterCachedMotor2.set(-power);
    }

    public void setHood(double pos){
        System.out.println(pos);
        hood.setPosition(Range.clip(pos, hoodLowerBound, hoodUpperBound));
    }

    public void update() {
        // Measure velocity
        currentVelocity = getCurrentVelo();
        smoothedVelocity = ALPHA * currentVelocity + (1 - ALPHA) * smoothedVelocity;

        long currTime = System.nanoTime();
        double dt = (currTime - prevTargetTime) / 1e9;
        double accel = 0.0;
        if (prevTargetTime != 0 && dt > 0) {
            accel = (targetVelocity - prevTargetVelocity) / dt;
        }
        prevTargetVelocity = targetVelocity;
        prevTargetTime = currTime;

        double outputPower;

        if (targetVelocity <= 0) {
            outputPower = 0;
            smoothedVelocity = 0;
        } else {
            outputPower = feedforward.calculate(targetVelocity, accel);
            if (enablePIDF){
                outputPower += pidf.calculate(smoothedVelocity, targetVelocity);
            }
        }

        setDirectPower(outputPower);
        upperGate.update();
        update_motors();
        turret1.update();
        turret2.update();
        hood.update();
    }

    public void update_motors(){
        shooterCachedMotor1.update();
        shooterCachedMotor2.update();
    }

    public double getTargetVelo() {
        return targetVelocity;
    }

    public double getCurrentVelo() {
        System.out.println("1: "+shooterEncoder1.getVelocity());
        System.out.println("2: "+shooterEncoder2.getVelocity());

        if(shooterEncoder1.getVelocity()<10){
                return Math.abs(shooterEncoder2.getVelocity());
        }
        else{
            return Math.abs(shooterEncoder1.getVelocity());
        }
    }

    // getters for vx and vy
    public double getVx() {
        return vx;
    }

    public double getVy() {
        return vy;
    }

    // getters for ax and ay
    public double getAx() {
        return ax;
    }

    public double getAy() {
        return ay;
    }

    // getter for angular velocity (rad/s)
    public double getOmega() {
        return omega;
    }

    @Configurable
    public static class Tables {
        public static double minVelocity = 1200;
        public static double getHoodPosition(double distance) {
            double increasehood = 0;
            return  -1.10011e-8 * Math.pow(distance, 4)
                    - 7.29035e-7 * Math.pow(distance, 3)
                    + 0.000614768 * Math.pow(distance, 2)
                    - 0.06158 * distance
                    + 2.35001 +increasehood;

        }
        public static double getShooterVelocity(double distance) {
            double increase = 0;
            double vel =  0.0000300474 * Math.pow(distance, 4)
                    - 0.0077187 * Math.pow(distance, 3)
                    + 0.686797 * Math.pow(distance, 2)
                    - 19.03258 * distance
                    + 1309.75735 +increase;
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

        public static double instantShotCompensation = 0.03;
    }
}
