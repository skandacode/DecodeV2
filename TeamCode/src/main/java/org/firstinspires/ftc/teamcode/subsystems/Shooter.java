package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import solverslib.controller.PIDFController;
import solverslib.controller.feedforwards.SimpleMotorFeedforward;
import solverslib.hardware.ServoEx;
import solverslib.hardware.motors.Motor;

@Configurable
public class Shooter {

    private Motor shooterMotor1, shooterMotor2;
    private ServoEx turret, hood;

    private ServoEx upperGate;

    VoltageSensor voltageSensor;

    private PIDFController pidf;
    private SimpleMotorFeedforward feedforward;

    private double targetVelocity = 0.0;
    private double currentVelocity = 0.0;

    // --- Flywheel PIDF coefficients ---
    public static double kP = 0.008;
    public static double kI = 0;
    public static double kD = 0;

    public static double kS = 0.115; // Static feedforward
    public static double kV = 0.00038; // Velocity feedforward

    public static boolean enablePIDF = true;

    // --- Turret bounds ---
    public static double turretUpperBound = 0.95;
    public static double turretLowerBound = 0.05;

    // --- Hood bounds ---

    public static double hoodLowerBound = 0.355;
    public static double hoodUpperBound = 0.82;

    // --- Low-pass filter coefficient (for smoothing) ---
    public static double ALPHA = 0.3;
    private double smoothedVelocity = 0.0;

    public enum Goal{
        RED (new Pose(-53, 63)),
        BLUE (new Pose(-53, -63));

        Pose position;
        Goal(Pose pose) {
            position = pose;
        }
    }

    public static double powerOffset = 0;
    public static double turretOffset = 0;

    public static double upperGateOpenPos = 0.89;
    public static double upperGateClosedPos = 0.73;

    private double prevX, prevY;
    private long prevPosTime;

    // add vx and vy fields
    private double vx = 0.0;
    private double vy = 0.0;

    public Shooter(HardwareMap hardwareMap) {
        shooterMotor1 = new Motor(hardwareMap, "outtakemotor1");
        shooterMotor2 = new Motor(hardwareMap, "outtakemotor2");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        upperGate = new ServoEx(hardwareMap, "upperGate");

        turret = new ServoEx(hardwareMap, "turret");
        hood = new ServoEx(hardwareMap, "hood");

        pidf = new PIDFController(kP, kI, kD, 0);
        feedforward = new SimpleMotorFeedforward(kS, kV);

        // initialize previous pos time to avoid large dt on first call
        prevPosTime = System.nanoTime();
        prevX = 0.0;
        prevY = 0.0;
    }

    public void setUpperGateOpen(boolean open){
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
        turret.setPosition(safePos);
    }

    public double convertDegreestoServoPos(double deg){
        return deg*0.003222222222222222+0.511111111;
    }

    public void aimAtTarget(Pose currPosition, Goal target){
        aimAtTarget(currPosition, target.position);
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

        // store to instance fields for external access if needed
        this.vx = computedVx;
        this.vy = computedVy;

        double precomputedDistance = getAngleDistance(currPosition, target)[1];

        Pose realTarget = new Pose(
                target.getX() - this.vx * ShooterTables.getBalltimeinair(precomputedDistance),
                target.getY() - this.vy * ShooterTables.getBalltimeinair(precomputedDistance),
                target.getHeading());


        double[] angleDistance = getAngleDistance(currPosition, realTarget);
        double angle = angleDistance[0];
        double distance = angleDistance[1];

        double servoPos = convertDegreestoServoPos(angle + turretOffset);

        double currVelo = getCurrentVelocity();

        servoPos = Range.clip(servoPos, turretLowerBound, turretUpperBound);


        setTurretPos(servoPos);
        setTargetVelocity(ShooterTables.getShooterVelocity(distance) + powerOffset);
        setHood(ShooterTables.getHoodPosition(distance) + ShooterTables.getHoodAngleChange(currVelo, distance));

        // update previous position/time for next velocity calculation
        prevX = currPosition.getX();
        prevY = currPosition.getY();
        prevPosTime = currTime;
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
        shooterMotor1.set(power);
        shooterMotor2.set(-power);
    }

    public void setHood(double pos){
        hood.setPosition(Range.clip(pos, hoodLowerBound, hoodUpperBound));
    }

    public void update() {
        // Measure velocity
        currentVelocity = Math.abs(shooterMotor2.getVelocity());
        smoothedVelocity = ALPHA * currentVelocity + (1 - ALPHA) * smoothedVelocity;

        double outputPower;

        if (targetVelocity <= 0) {
            outputPower = 0;
            smoothedVelocity = 0;
        } else {
            outputPower = feedforward.calculate(targetVelocity);
            if (enablePIDF){
                outputPower += pidf.calculate(smoothedVelocity, targetVelocity);
            }
        }

        setDirectPower(outputPower);
        shooterMotor1.update();
        shooterMotor2.update();
        upperGate.update();
        turret.update();
        hood.update();
    }

    public double getTargetVelo() {
        return targetVelocity;
    }

    // getters for vx and vy
    public double getVx() {
        return vx;
    }

    public double getVy() {
        return vy;
    }
}
