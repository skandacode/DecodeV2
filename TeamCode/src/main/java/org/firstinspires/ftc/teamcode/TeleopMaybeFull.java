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
public class TeleopMaybeFull extends LinearOpMode {
    Intakes intakes;
    Spindexer spindexer;
    Shooter shooter;
    Follower follower;

    public static Shooter.Goal target = Shooter.Goal.BLUE;
    public static double powerOffsetIncrements = 20;
    public static double turretOffsetIncrements = 2;

    public enum States{
        Intake,
        TwoInGood,
        OpenLowerGate, //if 2 go in good intake
        Increment1, //switch from Shoot1 to Intake2
        Wait1,
        Increment2, //switch from Intake2 to Intake3
        Wait2,
        Increment3, //switch from Intake3 to 4

        WaitForShoot,

        Kick1,
        ShootSpin1,
        ShootSpin2,


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

        GamepadKeys.Button slowModeButton = GamepadKeys.Button.RIGHT_BUMPER;
        GamepadKeys.Button positionResetButton = GamepadKeys.Button.TOUCHPAD;
        GamepadKeys.Button shooterButton = GamepadKeys.Button.B;
        GamepadKeys.Button backIntakeButton = GamepadKeys.Button.X;


        follower.setStartingPose(Position.pose);

        StateMachine stateMachine = new StateMachineBuilder()
                .state(States.Intake)
                .onEnter(()->{
                    intakes.setGoodIntakePower(1);
                    intakes.setBadIntakePower(0.2);

                    shooter.setUpperGateOpen(false);
                    spindexer.setLowerGateOpen(false);
                    spindexer.setKickerPos(false);
                    spindexer.setPosition(Spindexer.SpindexerPosition.Shoot1);
                })
                .transition(()->gamepadEx.getButton(shooterButton), States.OpenUpperGate)
                .transition(()->intakes.getGoodIntakeCurrent()>4, States.TwoInGood)
                .transition(()->gamepadEx.isDown(backIntakeButton), States.Increment1)


                .state(States.TwoInGood)
                .onEnter(()->{
                    spindexer.setLowerGateOpen(true);
                })
                .transition(()->gamepadEx.getButton(shooterButton), States.OpenUpperGate)

                .state(States.Increment1)
                .onEnter(()->{
                    spindexer.setPosition(Spindexer.SpindexerPosition.Intake2);
                    intakes.setBadIntakePower(1);
                })
                .transition(()->!gamepadEx.isDown(backIntakeButton), States.Intake)
                .transition(()->intakes.getBadIntakeDetected(), States.Wait1)

                .state(States.Wait1)
                .transitionTimed(0.2, States.Increment2)

                .state(States.Increment2)
                .onEnter(()->{
                    spindexer.setPosition(Spindexer.SpindexerPosition.Intake3);
                })
                .transition(()->intakes.getBadIntakeDetected(), States.Wait2)

                .state(States.Wait2)
                .transitionTimed(0.2, States.Increment3)

                .state(States.Increment3)
                .onEnter(()->{
                    spindexer.setPosition(Spindexer.SpindexerPosition.Intake4);
                })
                .transition(()->intakes.getBadIntakeDetected(), States.WaitForShoot)

                .state(States.WaitForShoot)
                .onEnter(()->{
                    spindexer.setPosition(Spindexer.SpindexerPosition.Shoot1);
                    intakes.setBadIntakePower(0.3);
                })
                .transition(()->gamepadEx.isDown(shooterButton), States.Kick1)

                .state(States.Kick1)
                .onEnter(()->{
                    spindexer.setLowerGateOpen(true);
                    shooter.setUpperGateOpen(true);
                    spindexer.setKickerPos(true);
                })
                .transitionTimed(0.3, States.ShootSpin1)

                .state(States.ShootSpin1)
                .onEnter(()->{
                    spindexer.setPosition(Spindexer.SpindexerPosition.Shoot2);
                })
                .transitionTimed(0.3, States.ShootSpin2)

                .state(States.ShootSpin2)
                .onEnter(()->{
                    spindexer.setPosition(Spindexer.SpindexerPosition.Shoot3);
                })
                .transitionTimed(0.3, States.Intake)

                .state(States.OpenUpperGate)
                .onEnter(()->{
                    spindexer.setLowerGateOpen(true);
                    shooter.setUpperGateOpen(true);
                })
                .transitionTimed(0.1, States.Shoot)

                .state(States.Shoot)
                .onEnter(()->{
                    spindexer.setKickerPos(true);
                })
                .transitionTimed(0.5, States.Intake)
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
        follower.startTeleopDrive();

        long lastLoopTime = System.nanoTime();
        while (opModeIsActive()) {
            if (controlhub != null) {
                controlhub.clearBulkCache();
            }else {
                for (LynxModule hub : allHubs) {
                    hub.clearBulkCache();
                }
            }

            gamepadEx.readButtons();
            follower.update();
            Position.pose = follower.getPose();
            telemetry.addData("Angle and distance:", Arrays.toString(shooter.getAngleDistance(Position.pose, target)));
            shooter.aimAtTarget(Position.pose, target);

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
                follower.setPose(new Pose(65, 0, Math.toRadians(180)));
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

            stateMachine.update();

            telemetry.addData("Current Pos", follower.getPose());
            telemetry.addData("Shooter Target", shooter.getTargetVelo());
            telemetry.addData("Shooter Velocity", shooter.getCurrentVelocity());

            long currentTime = System.nanoTime();
            double loopTime = (double) (currentTime - lastLoopTime) / 1_000_000.0;
            lastLoopTime = currentTime;
            telemetry.addData("Loop time", loopTime);
            intakes.update();
            spindexer.update();
            shooter.update();
            telemetry.update();
        }
    }
}
