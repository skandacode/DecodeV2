package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.pedroPathing.PanelsDrawing;
import org.firstinspires.ftc.teamcode.pedroPathing.PositionLogger;
import org.firstinspires.ftc.teamcode.subsystems.Intakes;
import org.firstinspires.ftc.teamcode.subsystems.Position;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.subsystems.Tilt;

import java.io.IOException;
import java.security.cert.PKIXRevocationChecker;
import java.util.Arrays;
import java.util.List;

import solverslib.gamepad.GamepadEx;
import solverslib.gamepad.GamepadKeys;
import solverslib.gamepad.ToggleButtonReader;

@Configurable
@TeleOp
public class TeleopMaybeFull extends LinearOpMode {
    Intakes intakes;
    Spindexer spindexer;
    Shooter shooter;
    Follower follower;
    Tilt tilt;

    public static Shooter.Goal target = Shooter.Goal.BLUE;
    public static double powerOffsetIncrements = 20;
    public static double turretOffsetIncrements = 2;

    public static double shootWaitTime = 0.28;

    private boolean is_split = false; //if 2 in good and other ball goes in from back
    private boolean using_spindexer = false;

    public static Pose relocalizePos = new Pose(60, -0, Math.toRadians(180));

    PositionLogger positionLogger;

    public enum States{
        Intake,
        Transitionto2,
        TwoInGood,

        Increment0,
        Increment1,
        Wait1,
        Increment2,
        DelayBit,
        Wait2,

        BeforeWaitForShoot,
        WaitForShoot,

        Kick1,//spindex shoot
        ShootSpin1,
        Kick2,
        ShootSpin2,
        Kick3,

        OpenUpperGate,//rapid 3
        Shoot,

        OpenUpperGate2,
        Shoot2,
        CollectBack,
        MoveBack,
        ShootBack,
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        try {
            positionLogger = new PositionLogger("TeleopFull.log");
        } catch (java.io.IOException e) {
            System.out.println("Logger failed to Initialize");
            positionLogger = null;
        }
        GamepadEx gamepadEx = new GamepadEx(gamepad1);
        GamepadEx gamepadEx2 = new GamepadEx(gamepad2);


        intakes = new Intakes(hardwareMap);
        spindexer = new Spindexer(hardwareMap);
        shooter = new Shooter(hardwareMap);
        follower = createFollower(hardwareMap);
        tilt = new Tilt(hardwareMap);

        GamepadKeys.Button slowModeButton = GamepadKeys.Button.RIGHT_BUMPER;
        GamepadKeys.Button positionResetButton = GamepadKeys.Button.TOUCHPAD;
        GamepadKeys.Button shooterButton = GamepadKeys.Button.B;
        GamepadKeys.Trigger backIntakeButton = GamepadKeys.Trigger.RIGHT_TRIGGER;
        GamepadKeys.Button stopIntakeButton = GamepadKeys.Button.A;
        GamepadKeys.Button restartIntakeButton = GamepadKeys.Button.Y;
        GamepadKeys.Button tiltButton = GamepadKeys.Button.LEFT_BUMPER;

        boolean[] backStopped = {false};

        follower.setStartingPose(Position.pose);

        StateMachine stateMachine = new StateMachineBuilder()
                .state(States.Intake)
                .onEnter(()->{
                    intakes.setGoodIntakePower(1);
                    if (backStopped[0]){
                        intakes.setBadIntakePower(0);
                    }else {
                        intakes.setBadIntakePower(0.5);
                    }
                    shooter.setUpperGateOpen(false);
                    spindexer.setLowerGateOpen(false);
                    spindexer.setKickerPos(false);
                    spindexer.setPosition(Spindexer.SpindexerPosition.Shoot1);
                    using_spindexer = false;
                    is_split = false;
                })
                .transition(()->gamepadEx.getButton(shooterButton), States.WaitForShoot)
                .transition(()->intakes.getGoodBeamBreakInside() && intakes.getGoodIntakeDetected(), States.Transitionto2)
                .transition(()->intakes.getBadBeamBreak(), States.Increment0)

                .state(States.Transitionto2)
                .onEnter(()->{
                    spindexer.setLowerGateOpen(true);
                    shooter.setUpperGateOpen(false);
                })
                .transitionTimed(0.1)

                .state(States.TwoInGood)
                .onEnter(()->{
                    spindexer.setLowerGateOpen(true);
                    shooter.setUpperGateOpen(false);
                })
                .transition(()->gamepadEx.getButton(shooterButton), States.WaitForShoot)
                .transition(()->gamepadEx.getButton(shooterButton), States.WaitForShoot)
                .transition(()->intakes.getGoodBeamBreakOutside(), States.BeforeWaitForShoot)
                .transition(()-> intakes.getBadBeamBreak(), States.BeforeWaitForShoot, ()-> is_split = true)

                .state(States.Increment0)
                .onEnter(()->{
                    spindexer.setPosition(Spindexer.SpindexerPosition.Intake2);
                    intakes.setBadIntakePower(1);
                    intakes.setGoodIntakePower(0.3);
                    using_spindexer = true;
                })
                .transitionTimed(0.3)
                .transition(()->gamepadEx.getButton(shooterButton), States.WaitForShoot)

                .state(States.Wait1)
                .onEnter(()->{
                    spindexer.setPosition(Spindexer.SpindexerPosition.Intake3);
                })
                .transitionTimed(shootWaitTime, States.Increment1)
                .transition(()->gamepadEx.getButton(shooterButton), States.WaitForShoot)

                .state(States.Increment1)
                .onEnter(()->{
                    spindexer.setPosition(Spindexer.SpindexerPosition.Intake3);
                })
                .transition(()->intakes.getBadIntakeDetected(), States.Wait2)
                .transition(()->gamepadEx.getButton(shooterButton), States.WaitForShoot)
                .transition(()->intakes.getGoodBeamBreakOutside() || intakes.getGoodBeamBreakInside(), States.DelayBit, ()->{
                    intakes.setGoodIntakePower(1);
                    intakes.setBadIntakePower(0.1);
                    spindexer.setPosition(Spindexer.SpindexerPosition.Shoot3);
                })

                .waitState(States.DelayBit, shootWaitTime)

                .state(States.Wait2)
                .onEnter(()->{
                    spindexer.setPosition(Spindexer.SpindexerPosition.Intake4);
                    intakes.setBadIntakePower(1);
                    intakes.setGoodIntakePower(0.3);
                })
                .transitionTimed(shootWaitTime, States.Increment2)
                .transition(()->gamepadEx.getButton(shooterButton), States.WaitForShoot)

                .state(States.Increment2)
                .onEnter(()->{
                    spindexer.setPosition(Spindexer.SpindexerPosition.Intake4);
                })
                .transition(()->intakes.getBadIntakeDetected(), States.WaitForShoot)
                .transition(()->gamepadEx.getButton(shooterButton), States.WaitForShoot)
                .transition(()->intakes.getGoodBeamBreakOutside(), States.BeforeWaitForShoot, ()->{
                    intakes.setGoodIntakePower(1);
                    intakes.setBadIntakePower(0.1);
                })

                .state(States.BeforeWaitForShoot)
                .onEnter(()->{
                    spindexer.setPosition(Spindexer.SpindexerPosition.Shoot1);
                    intakes.setGoodIntakePower(1);
                })
                .transitionTimed(0.3, States.WaitForShoot)

                .state(States.WaitForShoot)
                .onEnter(()->{
                    spindexer.setPosition(Spindexer.SpindexerPosition.Shoot1);
                    if (is_split || using_spindexer) {
                        intakes.setBadIntakePower(0.1);
                    }else{
                        intakes.setBadIntakePower(0);
                    }
                    if (gamepadEx.isDown(GamepadKeys.Button.Y)){
                        intakes.setGoodIntakePower(1);
                    }else {
                        intakes.setGoodIntakePower(0.1);
                    }
                })
                .transition(()->gamepadEx.isDown(shooterButton) && !(using_spindexer || is_split), States.OpenUpperGate)
                .transition(()->gamepadEx.isDown(shooterButton) && using_spindexer, States.Kick1)
                .transition(()->gamepadEx.isDown(shooterButton) && is_split, States.OpenUpperGate2)



                .state(States.OpenUpperGate)
                .onEnter(()->{
                    intakes.setGoodIntakePower(1);
                    spindexer.setLowerGateOpen(true);
                    shooter.setUpperGateOpen(true);
                })
                .transitionTimed(0.2)

                .state(States.Shoot)
                .onEnter(()->{
                    spindexer.setKickerPos(true);
                })
                .transitionTimed(0.4, States.Intake)

                .state(States.Kick1)
                .onEnter(()->{
                    spindexer.setPosition(Spindexer.SpindexerPosition.Shoot1);
                    spindexer.setLowerGateOpen(true);
                    shooter.setUpperGateOpen(true);
                    spindexer.setKickerPos(true);
                    intakes.setGoodIntakePower(1);
                })
                .transitionTimed(shootWaitTime, States.ShootSpin1)

                .state(States.ShootSpin1)
                .onEnter(()->{
                    spindexer.setPosition(Spindexer.SpindexerPosition.Shoot2);
                    spindexer.setKickerPos(false);
                })
                .transitionTimed(shootWaitTime, States.Kick2)

                .state(States.Kick2)
                .onEnter(()->{
                    spindexer.setKickerPos(true);
                })
                .transitionTimed(shootWaitTime, States.ShootSpin2)

                .state(States.ShootSpin2)
                .onEnter(()->{
                    spindexer.setPosition(Spindexer.SpindexerPosition.Shoot3);
                    spindexer.setKickerPos(false);
                })
                .transitionTimed(shootWaitTime, States.Kick3)

                .state(States.Kick3)
                .onEnter(()->{
                    spindexer.setKickerPos(true);
                })
                .transitionTimed(shootWaitTime, States.Intake)

                .state(States.OpenUpperGate2)
                .onEnter(()->{
                    intakes.setGoodIntakePower(1);
                    spindexer.setLowerGateOpen(true);
                    shooter.setUpperGateOpen(true);
                })
                .transitionTimed(0.15)

                .state(States.Shoot2)
                .onEnter(()->{
                    spindexer.setKickerPos(true);
                })
                .transitionTimed(0.15)

                .state(States.CollectBack)
                .onEnter(()->{
                    intakes.setGoodIntakePower(0.1);
                    spindexer.setKickerPos(false);
                    spindexer.setPosition(Spindexer.SpindexerPosition.Intake2);
                    intakes.setBadIntakePower(1);
                })
                .transitionTimed(0.4)

                .state(States.MoveBack)
                .onEnter(()->{
                    intakes.setGoodIntakePower(1);
                    spindexer.setPosition(Spindexer.SpindexerPosition.Shoot2);
                    intakes.setBadIntakePower(0.1);
                })
                .transitionTimed(0.3)

                .state(States.ShootBack)
                .onEnter(()->{
                    spindexer.setKickerPos(true);
                })
                .transitionTimed(shootWaitTime, States.Intake)
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

        boolean intakeStopped = false;
        boolean intakeReversed = false;
        boolean tilted = false;
        double goodMotorPowerBeforeStop = 0;
        double badMotorPowerBeforeStop = 0;
        boolean prevTriggerPressed = false;

        tilt.retract();
        while (opModeIsActive()) {
            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }

            gamepadEx.readButtons();
            gamepadEx2.readButtons();
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
                follower.setPose(relocalizePos);
                Shooter.powerOffset=0;
                Shooter.turretOffset=0;
            }
            if (gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_DOWN) || gamepadEx2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
                Shooter.powerOffset -= powerOffsetIncrements;
            }
            if (gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_LEFT) || gamepadEx2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)){
                Shooter.turretOffset -= turretOffsetIncrements;
            }
            if (gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT) || gamepadEx2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)){
                Shooter.turretOffset += turretOffsetIncrements;
            }
            if (gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_UP) || gamepadEx2.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
                Shooter.powerOffset += powerOffsetIncrements;
            }

            if (gamepadEx.wasJustPressed(GamepadKeys.Button.OPTIONS) && !intakeStopped){
                goodMotorPowerBeforeStop = intakes.goodPower;
                badMotorPowerBeforeStop = intakes.badPower;
                intakes.setGoodIntakePower(-0.5);
                intakes.setBadIntakePower(-0.5);
                intakeReversed = true;
            }
            if (gamepadEx.wasJustReleased(GamepadKeys.Button.OPTIONS)){
                intakes.setGoodIntakePower(goodMotorPowerBeforeStop);
                intakes.setBadIntakePower(goodMotorPowerBeforeStop);
                intakeReversed = false;
            }

            if (gamepadEx.wasJustPressed(stopIntakeButton) && !intakeReversed){
                if (intakes.goodPower != 0 || intakes.badPower != 0) {
                    goodMotorPowerBeforeStop = intakes.goodPower;
                    badMotorPowerBeforeStop = intakes.badPower;
                }
                intakeStopped = true;
            }
            if (gamepadEx.isDown(shooterButton)){
                intakeStopped = false;
            }
            if ((gamepadEx.wasJustPressed(restartIntakeButton)) && stateMachine.getState() != States.WaitForShoot){
                intakeStopped = false;
                intakes.setGoodIntakePower(goodMotorPowerBeforeStop);
                intakes.setBadIntakePower(badMotorPowerBeforeStop);
            }

            if (!intakeStopped && !tilted) {
                stateMachine.update();
            }else{
                intakes.setGoodIntakePower(0);
                intakes.setBadIntakePower(0);
            }
            if (gamepadEx.wasJustPressed(GamepadKeys.Button.X)){
                using_spindexer = true;
            }
            if (tilted){
                shooter.setDirectPower(0);
            }
            if (gamepadEx.wasJustPressed(tiltButton)) {
                tilted = !tilted;
                if (tilted) {
                    tilt.tilt();
                } else {
                    tilt.retract();
                }
            }

            boolean currTrigger = gamepadEx.getTrigger(backIntakeButton)>0.5;
            if (currTrigger != prevTriggerPressed){
                backStopped[0] = currTrigger;
            }
            prevTriggerPressed = currTrigger;

            telemetry.addData("Current Pos", follower.getPose());
            telemetry.addData("Shooter Target", shooter.getTargetVelo());
            telemetry.addData("Shooter Velocity", shooter.getCurrentVelocity());
            telemetry.addData("State", stateMachine.getState());
            telemetry.addData("is_split", is_split);
            telemetry.addData("using_spindexer", using_spindexer);



            long currentTime = System.nanoTime();
            double loopTime = (double) (currentTime - lastLoopTime) / 1_000_000.0;
            lastLoopTime = currentTime;
            telemetry.addData("Loop time", loopTime);
            intakes.update();
            spindexer.update();
            if (!tilted) {
                shooter.update();
            }else{
                shooter.update_motors();
            }
            tilt.update();
            telemetry.update();
            try {
                if (positionLogger != null){
                    positionLogger.logPose(follower.getPose());
                }
            } catch (IOException ignored) {

            }
            PanelsDrawing.drawDebug(follower);
        }
        try {
            if (positionLogger != null){
                positionLogger.close();
            }
        } catch (IOException ignored) {

        }
    }
}
