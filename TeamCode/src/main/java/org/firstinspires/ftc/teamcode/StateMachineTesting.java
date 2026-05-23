package org.firstinspires.ftc.teamcode;


import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.teamcode.subsystems.Intakes;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Spindexer;

import java.util.List;

@Configurable
@TeleOp
public class StateMachineTesting extends LinearOpMode {
    public static int[] shootorder = {0, 1, 2};
    Intakes intakes;
    Shooter shooter;
    Spindexer spindexer;
    public static boolean shooterButton = false;
    public static double shootWaitTime = 0.28;
    public static double pulseTime = 0.05;
    public static double intakeStallTime = 0.1;
    public static double preKickerTime = 0.3;
    public static double kickTime = 0.4;


    public static boolean rapidFire = false;
    public static double shooterVelocity = 1800;

    public enum States{
        Intake,
        TransferOff,
        BeforePulseOut,
        PulseOut,
        PulseIn,
        HoldBalls,
        PreShoot,
        OpenUpperGate,
        Shoot,
    }



    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        intakes = new Intakes(hardwareMap);
        shooter = new Shooter(hardwareMap);
        spindexer = new Spindexer(hardwareMap);

        while (opModeInInit()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();
            telemetry.update();
        }

        waitForStart();
        System.out.println("started");

        StateMachine stateMachine = new StateMachineBuilder()
                .state(TeleopOnlyRapidFAR.States.Intake)
                .onEnter(() -> {
                    intakes.setGoodIntakePower(1);
                    shooter.setUpperGateOpen(false);
                    spindexer.setLowerGateOpen(true);
                    spindexer.setKickerPos(false);
                    spindexer.setPosition(Spindexer.SpindexerPosition.Shoot0);
                })
                .transition(() -> shooterButton, TeleopOnlyRapidFAR.States.OpenUpperGate)
                .transition(() -> intakes.getGoodBeamBreakInside() && intakes.getGoodIntakeDetected(), TeleopOnlyRapidFAR.States.TransferOff)

                .state(TeleopOnlyRapidFAR.States.TransferOff)
                .onEnter(() -> intakes.setTransferIntakePower(0.3))
                .transition(() -> intakes.getGoodBeamBreakOutside() && intakes.getGoodBeamBreakInside(), TeleopOnlyRapidFAR.States.BeforePulseOut)
                .transition(() -> shooterButton, TeleopOnlyRapidFAR.States.OpenUpperGate)

                .state(TeleopOnlyRapidFAR.States.BeforePulseOut)
                .onEnter(() -> intakes.setFrontIntakePower(1))
                .transitionTimed(0.3)
                .transition(() -> shooterButton, TeleopOnlyRapidFAR.States.OpenUpperGate)

                .state(TeleopOnlyRapidFAR.States.PulseOut)
                .onEnter(() -> intakes.setFrontIntakePower(-0.1))
                .transitionTimed(pulseTime)
                .transition(() -> shooterButton, TeleopOnlyRapidFAR.States.OpenUpperGate)

                .state(TeleopOnlyRapidFAR.States.PulseIn)
                .onEnter(() -> intakes.setFrontIntakePower(1))
                .transitionTimed(0.2)
                .transition(() -> shooterButton, TeleopOnlyRapidFAR.States.OpenUpperGate)

                .state(TeleopOnlyRapidFAR.States.HoldBalls)
                .onEnter(() -> intakes.setGoodIntakePower(0.1))
                .loop(() -> {
                    if ((intakes.getGoodBeamBreakOutside() && intakes.getGoodBeamBreakInside())) {
                        intakes.setGoodIntakePower(0.1);
                    } else {
                        intakes.setGoodIntakePower(1);
                    }
                })
                .transition(() -> shooterButton, TeleopOnlyRapidFAR.States.PreShoot)

                .state(TeleopOnlyRapidFAR.States.PreShoot)
                .onEnter(() -> {
                    intakes.setGoodIntakePower(0.75);
                })
                .transitionTimed(intakeStallTime, TeleopOnlyRapidFAR.States.OpenUpperGate)
                .state(TeleopOnlyRapidFAR.States.OpenUpperGate)
                .onEnter(() -> {
                    shooter.setUpperGateOpen(true);
                })
                .transitionTimed(preKickerTime, TeleopOnlyRapidFAR.States.Shoot)
                .state(TeleopOnlyRapidFAR.States.Shoot)
                .onEnter(() -> {
                    spindexer.setKickerPos(true);
                })
                .transitionTimed(kickTime, TeleopOnlyRapidFAR.States.Intake)
                .build();

        stateMachine.start();
        shooter.setTargetVelocity(shooterVelocity);

        while (opModeIsActive()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();
            stateMachine.update();
            intakes.update();
            shooter.update();
            spindexer.update();
            telemetry.addData("State: ", stateMachine.getState());
            telemetry.addData("Shooter Target", shooter.getTargetVelo());
            telemetry.addData("Shooter Velocity", shooter.getCurrentVelocity());
            telemetry.addData("Good Inside Intake beam break", intakes.getGoodBeamBreakInside());
            telemetry.addData("Good Outside Intake beam break", intakes.getGoodBeamBreakOutside());
            telemetry.addData("Good intake distance", intakes.getGoodIntakeDistance());
            telemetry.addData("Spindexer Position", spindexer.getCurrentPosition());

            telemetry.addData("Spindexer kick", spindexer.is_kick);
            telemetry.update();
        }
    }
}

