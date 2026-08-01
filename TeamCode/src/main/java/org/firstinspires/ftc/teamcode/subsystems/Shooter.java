package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.utils.PIDFController;
import org.firstinspires.ftc.teamcode.utils.SimpleMotorFeedforward;
import org.firstinspires.ftc.teamcode.utils.CachedServo;
import org.firstinspires.ftc.teamcode.utils.CachedMotor;

@Configurable
public class Shooter {

    private CachedMotor shooterCachedMotor1, shooterCachedMotor2, shooterEncoder1, shooterEncoder2;
    private CachedServo turret1, turret2;
    private Servo hood;

    private CachedServo upperGate;

    VoltageSensor voltageSensor;

    private PIDFController pidf;
    private SimpleMotorFeedforward feedforward;

    private double targetVelocity = 0.0;
    private double currentVelocity = 0.0;
    public static double diffTurret = 0.001;
    // --- Flywheel PIDF coefficients ---
    public static double kP = 0.005;

    public static double kS = 0.0613641; // Static feedforward
    public static double kV = 0.000375058; // Velocity feedforward

    public static boolean enablePIDF = true;

    // --- Turret bounds ---
    public static double turretUpperBound = 0.95;
    public static double turretLowerBound = 0.05;

    // --- Hood bounds ---

    public static double hoodLowerBound = 0.48;
    public static double hoodUpperBound = 0.85;

    // --- Low-pass filter coefficient (for smoothing) ---
    public static double ALPHA = 0.3;
    private double smoothedVelocity = 0.0;

    public static Pose RedGoalPose = new Pose(-70, 62);
    public static Pose BlueGoalPose = new Pose(-67, -62);

    public enum Goal{
        RED (RedGoalPose),
        BLUE (BlueGoalPose);

        Pose position;
        Goal(Pose pose) {
            position = pose;
        }
    }

    public static double powerOffset = 0;
    public static double hoodOffset = 0;
    public static double turretOffset = 0;
    public static double limelightOffset = 0;


    public static double upperGateOpenPos = 0.68;
    public static double upperGateClosedPos = 0.56;
    private double prevTargetVelocity = 0.0;
    private long prevTargetTime;

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

        hood = hardwareMap.servo.get("hood");

        pidf = new PIDFController(kP, 0, 0, 0);
        feedforward = new SimpleMotorFeedforward(kS, kV);

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
        double dx = target.x()-currPosition.x();
        double dy = target.y()-currPosition.y();
        double angle = Math.atan2(dy, dx);
        double turretAngle = Math.toDegrees(-angle + currPosition.heading());

        while (Math.abs(turretAngle)>=180){
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
        turret1.setPosition(safePos+diffTurret);
        turret2.setPosition(safePos-diffTurret);

    }

    public double convertDegreestoServoPos(double deg){
        return deg*-0.003111111111111111+0.5;
    }

    public void aimAtTarget(Pose currPosition, Goal target){
        aimAtTarget(currPosition, target.position);
    }

    public void aimTurret(Pose currPosition, Goal target){
        double[] angleDistance = getAngleDistance(currPosition, target);
        double angle = angleDistance[0];

        double servoPos = convertDegreestoServoPos(angle + turretOffset + limelightOffset);

        servoPos = Range.clip(servoPos, turretLowerBound, turretUpperBound);

        setTurretPos(servoPos);
    }

    public void aimAtTarget(Pose currPosition, Pose target){
        double[] angleDistance = getAngleDistance(currPosition, target);
        double angle = angleDistance[0];
        double distance = angleDistance[1];

        double servoPos = convertDegreestoServoPos(angle + turretOffset + limelightOffset);

        servoPos = Range.clip(servoPos, turretLowerBound, turretUpperBound);

        setTurretPos(servoPos);
        setTargetVelocity(Tables.getShooterVelocity(distance) + powerOffset);
        setHood(Tables.getHoodPosition(distance) + hoodOffset);
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
        System.out.println("hood"+pos);
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
                double error = (targetVelocity - currentVelocity);

                if (Math.abs(error) > 60)
                    outputPower = Math.signum(error);
                else
                    outputPower += pidf.calculate(smoothedVelocity, targetVelocity);
            }
        }

        setDirectPower(Math.max(outputPower,0));
        upperGate.update();
        update_motors();
        turret1.update();
        turret2.update();
    }

    public void update_motors(){
        shooterCachedMotor1.update();
        shooterCachedMotor2.update();
    }

    public double getTargetVelo() {
        return targetVelocity;
    }

    public double getCurrentVelo() {
        System.out.println("Shooter Powers: "+shooterEncoder1.getVelocity() + ", "+shooterEncoder2.getVelocity());

        if(shooterEncoder1.getVelocity()<10){
            return Math.abs(shooterEncoder2.getVelocity());
        }
        else{
            return Math.abs(shooterEncoder1.getVelocity());
        }
    }

    @Configurable
    public static class Tables {
        public static double minVelocity = 1200;
        public static double getHoodPosition(double distance) {
            double increasehood = 0;
            double hood =  1.82496e-8 * Math.pow(distance, 4)
                    - 0.00000387675 * Math.pow(distance, 3)
                    + 0.000164823 * Math.pow(distance, 2)
                    + 0.0128326 * distance
                    - 0.148405 +increasehood;
            return Math.min(hood, 0.8);

        }
        public static double getShooterVelocity(double distance) {
            double increase = 0;
            double vel =  -0.0000108852 * Math.pow(distance, 4)
                    + 0.00192886 * Math.pow(distance, 3)
                    - 0.0697063 * Math.pow(distance, 2)
                    + 5.6394 * distance
                    + 950.93742 +increase;
            return Math.max(minVelocity, vel);
        }
    }
}