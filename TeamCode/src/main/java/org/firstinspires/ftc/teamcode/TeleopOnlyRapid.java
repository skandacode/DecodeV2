package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.subsystems.Intakes;
import org.firstinspires.ftc.teamcode.subsystems.Position;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

import java.util.Arrays;
import java.util.List;

import solverslib.gamepad.GamepadEx;
import solverslib.gamepad.GamepadKeys;
@Configurable
@TeleOp
public class TeleopOnlyRapid extends LinearOpMode {
    Intakes intakes;
    Spindexer spindexer;
    Shooter shooter;
    Follower follower;

    public static Shooter.Goal target = Shooter.Goal.BLUE;
    public static double powerOffsetIncrements = 20;
    public static double turretOffsetIncrements = 2;//0000000000 why not 0 bum bum bum

    public static double targetX = -53;
    public static double targetY = 63;


    public enum States{
        Intake,
        HoldBalls,
        OpenUpperGate,
        Shoot,
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        LynxModule controlhub = null;
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            if (hub.isParent()) {
                controlhub = hub;
                break;
            }
        }

        GamepadEx gamepadEx = new GamepadEx(gamepad1);

        intakes = new Intakes(hardwareMap);
        spindexer = new Spindexer(hardwareMap);
        shooter = new Shooter(hardwareMap);
        follower = createFollower(hardwareMap);

        ElapsedTime currentMonitoringTime = new ElapsedTime();

        GamepadKeys.Button slowModeButton = GamepadKeys.Button.RIGHT_BUMPER;
        GamepadKeys.Button positionResetButton = GamepadKeys.Button.LEFT_BUMPER;
        GamepadKeys.Button shooterButton = GamepadKeys.Button.B;
        GamepadKeys.Button stopIntakeButton = GamepadKeys.Button.A;
        GamepadKeys.Button restartIntake = GamepadKeys.Button.Y;



        follower.setStartingPose(Position.pose);

        StateMachine stateMachine = new StateMachineBuilder()
                .state(States.Intake)
                .onEnter(()->{
                    intakes.setGoodIntakePower(1);
                    shooter.setUpperGateOpen(false);
                    spindexer.setLowerGateOpen(true);
                    spindexer.setKickerPos(false);
                    spindexer.setPosition(Spindexer.SpindexerPosition.Shoot1);
                })
                .transition(()->gamepadEx.getButton(stopIntakeButton), States.HoldBalls)
                .transition(()->gamepadEx.getButton(shooterButton), States.OpenUpperGate)

                .state(States.HoldBalls)
                .onEnter(()->intakes.setGoodIntakePower(0.1))
                .transition(()->gamepadEx.getButton(shooterButton), States.OpenUpperGate)
                .transition(()->gamepadEx.getButton(restartIntake), States.Intake)


                .state(States.OpenUpperGate)
                .onEnter(()->{
                    shooter.setUpperGateOpen(true);
                    intakes.setGoodIntakePower(1);
                })
                .transitionTimed(0.4, States.Shoot)
                .state(States.Shoot)
                .onEnter(()->{
                    spindexer.setKickerPos(true);
                })
                .transitionTimed(0.3, States.Intake)
                .build();

        StateMachine clearMachine = new StateMachineBuilder()
                .state("IDLE")
                .transition(()->gamepad1.touchpad)

                .state("Eject1")
                .onEnter(()->{
                    spindexer.setPosition(Spindexer.SpindexerPosition.Shoot1);
                    intakes.setBadIntakePower(-1);
                    intakes.setGoodIntakePower(-1);
                })
                .transitionTimed(0.5)

                .state("Eject2")
                .onEnter(()->{
                    spindexer.setPosition(Spindexer.SpindexerPosition.Intake3);
                    intakes.setBadIntakePower(-1);
                    intakes.setGoodIntakePower(-1);
                })
                .transitionTimed(0.5)

                .state("Eject3")
                .onEnter(()->{
                    spindexer.setPosition(Spindexer.SpindexerPosition.Shoot2);
                    intakes.setBadIntakePower(-1);
                    intakes.setGoodIntakePower(-1);
                })
                .transitionTimed(0.5, "IDLE", ()->{
                    intakes.setGoodIntakePower(1);
                    shooter.setUpperGateOpen(false);
                    spindexer.setLowerGateOpen(true);
                    spindexer.setKickerPos(false);
                    spindexer.setPosition(Spindexer.SpindexerPosition.Shoot1);
                })
                .build();



        while (opModeInInit()) {
            for (LynxModule hub : allHubs) hub.clearBulkCache();
            follower.update();
            if (gamepad1.a) {
                target = Shooter.Goal.BLUE;
            }
            if (gamepad1.b) {
                target = Shooter.Goal.RED;
            }
            telemetry.addData("Shooter Target", target);
            telemetry.addData("Current Pos", follower.getPose());
            telemetry.update();
        }

        waitForStart();

        stateMachine.start();
        clearMachine.start();

        follower.startTeleopDrive();

        long lastLoopTime = System.nanoTime();
        double prevGoodIntakeCurrent = 0.0;
        while (opModeIsActive()) {
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
            if (currentMonitoringTime.milliseconds()>150){
                intakes.updateCurrent();
                currentMonitoringTime.reset();
            }

            gamepadEx.readButtons();
            follower.update();
            Position.pose = follower.getPose();
            telemetry.addData("Angle and distance:", Arrays.toString(shooter.getAngleDistance(Position.pose, target)));
            shooter.aimAtTarget(Position.pose, target);

            //shooter.aimAtTarget(Position.pose, new Pose(targetX, targetY));

            double forward = gamepadEx.getLeftY();
            double strafe = gamepadEx.getLeftX();
            double turn = gamepadEx.getRightX();
            if (gamepadEx.getButton(slowModeButton)) {
                forward *= 0.3;
                strafe *= 0.3;
                turn *= 0.3;
            }

            follower.setTeleOpDrive(forward, -1*strafe, -1*turn, true);

            if (gamepadEx.wasJustPressed(positionResetButton)){
                follower.setPose(new Pose(70, 0, Math.toRadians(180)));
                Shooter.powerOffset=0;
                Shooter.turretOffset=0;
            }
            if (gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
                Shooter.powerOffset -= powerOffsetIncrements;
            }
            if (gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)){
                Shooter.turretOffset -= turretOffsetIncrements;
            }
            if (gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)){
                Shooter.turretOffset += turretOffsetIncrements;
            }
            if (gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
                Shooter.powerOffset += powerOffsetIncrements;
            }
            if (gamepadEx.isDown(GamepadKeys.Button.OPTIONS)){
                intakes.setBadIntakePower(-1);
            }else{
                intakes.setBadIntakePower(-0);
            }

            if (intakes.getGoodIntakeCurrent()>Intakes.goodIntake3ThreshCurrent){
                if (prevGoodIntakeCurrent<Intakes.goodIntake3ThreshCurrent){
                    gamepad1.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);
                }
            }else{
                gamepad1.stopRumble();
            }
            stateMachine.update();
            clearMachine.update();

            telemetry.addData("Current Pos", follower.getPose());
            telemetry.addData("Shooter Target", shooter.getTargetVelo());
            telemetry.addData("Shooter Velocity", shooter.getCurrentVelocity());
            telemetry.addData("Spindexer kick", spindexer.is_kick);
            telemetry.addData("Statemachine State", stateMachine.getState());



            long currentTime = System.nanoTime();
            double loopTime = (double) (currentTime - lastLoopTime) / 1_000_000.0;
            lastLoopTime = currentTime;
            telemetry.addData("Loop time", loopTime);
            intakes.update();
            spindexer.update();
            shooter.update();
            telemetry.update();
            prevGoodIntakeCurrent = intakes.getGoodIntakeCurrent();
        }
    }
}
