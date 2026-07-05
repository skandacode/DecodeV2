package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.ivy.CommandBuilder;
import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.firstinspires.ftc.teamcode.utils.CommandOpMode;

public class Auto extends CommandOpMode {
    private Robot robot;
    private Paths paths;
    private final Alliance alliance;

    public Auto(Alliance alliance) {
        this.alliance = alliance;
    }

    @Override
    public void init() {
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        robot = new Robot(hardwareMap, alliance);
        paths = new Paths(robot.follower, alliance);
        robot.follower.setStartingPose(paths.start);

        robot.shooter.setUpperGate(false);
        robot.shooter.setTurretPos(robot.shooter.convertDegreestoServoPos(0));
        robot.transfer.setLowerGate(true);
        robot.transfer.setKicker(false);
        robot.intake.setPower(0);
        robot.update();
    }

    @Override
    public void init_loop() {
        robot.update();

        telemetry.addData("Init Pose", robot.follower.getPose());
        telemetry.update();
    }

    @Override
    public void start() {
        robot.follower.setStartingPose(paths.start);
        robot.transfer.setLowerGate(true);
        robot.shooter.setUpperGate(false);
        robot.shooter.setTurretPos(robot.shooter.convertDegreestoServoPos(0));
        robot.intake.setPower(0);
        robot.transfer.setKicker(false);

        schedule(
                infinite(() -> {
                    Robot.endPose = robot.follower.getPose();
                    robot.update();
                }),
                sequential(
                        preload(),
                        spike1(),
                        spike2(),
                        gate1(),
                        gate2(),
                        gate3(),
                        gate4()
                )
        );
    }

    @Override
    public void stop() {
        if (robot != null) {
            robot.saveEnd();
            robot.transfer.setLowerGate(true);
        }
        super.stop();
    }

    private CommandBuilder preload() {
        return sequential(
                prepareScore(76, -78, 0.56, 1440),
                paths.preload(),
                waitMs(1700.0),
                openGate(),
                waitMs(100.0),
                kick(),
                waitMs(300.0),
                resetAfterScore()
        );
    }

    private CommandBuilder spike1() {
        return sequential(
                intake(paths.intakeSpike1(), 2000.0),
                prepareScore(90, -95, 0.53, 1420),
                paths.scoreSpike1(),
                waitMs(1500.0),
                openGate(),
                waitMs(100.0),
                kick(),
                waitMs(430.0),
                resetAfterScore()
        );
    }

    private CommandBuilder spike2() {
        return sequential(
                intake(paths.intakeSpike2(), 2000.0),
                prepareScore(64, -65, 0.54, 1430),
                paths.scoreSpike2(),
                waitMs(1460.0),
                openGate(),
                waitMs(100.0),
                kick(),
                waitMs(300.0),
                resetAfterScore()
        );
    }

    private CommandBuilder gate1() {
        return sequential(
                intake(paths.intakeGate1(0.4), 2500.0),
                waitMs(300.0),
                prepareScore(78, -76, 0.54, 1440),
                paths.scoreGate(),
                waitMs(1500.0),
                openGate(),
                waitMs(100.0),
                kick(),
                waitMs(300.0),
                resetAfterScore()
        );
    }

    private CommandBuilder gate2() {
        return sequential(
                intake(paths.intakeGate2(0.6), 2500.0),
                waitMs(300.0),
                prepareScore(75, -78, 0.54, 1430),
                paths.scoreGate(),
                waitMs(1400.0),
                openGate(),
                waitMs(100.0),
                kick(),
                waitMs(300.0),
                resetAfterScore()
        );
    }

    private CommandBuilder gate3() {
        return sequential(
                intake(paths.intakeGate3(0.6), 2100.0),
                waitMs(300.0),
                prepareScore(75, -77, 0.54, 1430),
                paths.scoreGate(),
                waitMs(1500.0),
                openGate(),
                waitMs(100.0),
                kick(),
                waitMs(300.0),
                resetAfterScore()
        );
    }

    private CommandBuilder gate4() {
        return sequential(
                intake(paths.intakeGate4(0.6), 2500.0),
                waitMs(300.0),
                prepareScore(75, -80, 0.52, 1430),
                paths.scoreGate(),
                waitMs(1500.0),
                openGate(),
                waitMs(100.0),
                kick(),
                waitMs(300.0),
                resetAfterScore()
        );
    }

    private CommandBuilder intake(CommandBuilder path, double timeoutMs) {
        return sequential(
                prepareIntake(),
                path.raceWith(
                        waitUntil(this::full),
                        waitMs(timeoutMs)
                ),
                waitMs(50.0),
                stopIntake()
        );
    }

    private CommandBuilder prepareScore(double blueTurretDeg, double redTurretDeg, double hood, double velocity) {
        return instant(() -> {
            robot.transfer.setLowerGate(true);
            robot.intake.setPower(1);
            robot.transfer.setKicker(false);
            robot.shooter.setUpperGate(false);
            robot.shooter.setHood(hood);
            robot.shooter.setTargetVelocity(velocity);
            robot.shooter.setTurretPos(robot.shooter.convertDegreestoServoPos(alliance == Alliance.BLUE ? blueTurretDeg : redTurretDeg));
        });
    }

    private CommandBuilder prepareIntake() {
        return instant(() -> {
            robot.transfer.setKicker(false);
            robot.shooter.setUpperGate(false);
            robot.transfer.setLowerGate(true);
            robot.intake.setPower(1);
        });
    }

    private CommandBuilder stopIntake() {
        return instant(() -> robot.intake.setPower(0));
    }

    private CommandBuilder openGate() {
        return instant(() -> robot.shooter.setUpperGate(true));
    }

    private CommandBuilder kick() {
        return instant(() -> robot.transfer.setKicker(true));
    }

    private CommandBuilder resetAfterScore() {
        return instant(() -> {
            robot.transfer.setKicker(false);
            robot.shooter.setUpperGate(false);
        });
    }

    private boolean full() {
        return robot.intake.getBeamBreakOutside()
                && robot.intake.getBeamBreakInside()
                && robot.intake.getDetected();
    }
}
