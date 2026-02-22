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
    public static double intakeWaitTime = 0.35;


    public static boolean rapidFire = false;
    public static double shooterVelocity = 1000;

    public enum States{
        BeforeIntake,
        Intake,


        Increment1,
        Wait1,
        Increment2,
        Wait2,

        BeforeWaitForShoot,
        WaitForShoot,

        Kick1,//spindex shoot
        ShootSpin1,
        Kick2,
        ShootSpin2,
        Kick3,

        OpenUpperGate,//rapid 3
        Shoot
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
                .state(States.BeforeIntake)
                .loop(()->{
                    if (rapidFire) {
                        if (intakes.getGoodBeamBreakOutside() && intakes.getGoodBeamBreakInside() && intakes.getGoodIntakeDetected()) {
                            System.out.println("all detected");
                            intakes.setGoodIntakePower(0.2);
                        } else {
                            intakes.setGoodIntakePower(1);
                        }
                    }else{
                        intakes.setGoodIntakePower(1);
                    }
                    intakes.setBadIntakePower(-0.2);
                    shooter.setUpperGateOpen(false);
                    spindexer.setLowerGateOpen(rapidFire);
                    spindexer.setKickerPos(false);
                    spindexer.setPosition(Spindexer.SpindexerPosition.Shoot2);
                })
                .transition(()->shooterButton, States.WaitForShoot)
                .transitionTimed(0.4, States.Intake)


                .state(States.Intake)
                .loop(()->{
                    if (rapidFire) {
                        if (intakes.getGoodBeamBreakOutside() && intakes.getGoodBeamBreakInside() && intakes.getGoodIntakeDetected()) {
                            System.out.println("all detected");
                            intakes.setGoodIntakePower(0.2);
                        } else {
                            intakes.setGoodIntakePower(1);
                        }
                    }else{
                        intakes.setGoodIntakePower(1);
                    }
                    intakes.setBadIntakePower(-0.2);
                    shooter.setUpperGateOpen(false);
                    spindexer.setLowerGateOpen(rapidFire);
                    spindexer.setKickerPos(false);
                    spindexer.setPosition(Spindexer.SpindexerPosition.Shoot2);
                })
                .transition(()->shooterButton, States.WaitForShoot)
                .transition(()->!rapidFire && intakes.getGoodIntakeDetected(), States.Wait1)

                .state(States.Wait1)
                .onEnter(()->{
                    spindexer.setPosition(Spindexer.SpindexerPosition.Shoot1);
                })
                .transitionTimed(intakeWaitTime, States.Increment1)
                .transition(()->shooterButton, States.WaitForShoot)

                .state(States.Increment1)
                .onEnter(()->{
                    spindexer.setPosition(Spindexer.SpindexerPosition.Shoot1);
                })
                .transition(()->intakes.getGoodIntakeDetected(), States.Wait2)
                .transition(()->shooterButton, States.WaitForShoot)

                .state(States.Wait2)
                .onEnter(()->{
                    spindexer.setPosition(Spindexer.SpindexerPosition.Shoot0);
                    intakes.setBadIntakePower(0.3);
                })
                .transitionTimed(intakeWaitTime, States.Increment2)
                .transition(()->shooterButton, States.WaitForShoot)

                .state(States.Increment2)
                .onEnter(()->{
                    spindexer.setPosition(Spindexer.SpindexerPosition.Shoot0);
                })
                .transition(()->intakes.getGoodIntakeDetected(), States.WaitForShoot)
                .transition(()->shooterButton, States.WaitForShoot)

                .state(States.WaitForShoot)
                .onEnter(()->{
                    if (shootorder[0] == 2) {
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot0);
                    }else if (shootorder[0] == 1) {
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot1);
                    }else if (shootorder[0] == 0) {
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot2);
                    }
                })
                .transition(()->shooterButton && rapidFire, States.OpenUpperGate)
                .transition(()->shooterButton && !rapidFire, States.Kick1)
                .onExit(()->{
                    shooterButton = false;
                })


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
                    if (shootorder[0] == 2) {
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot0);
                    }else if (shootorder[0] == 1) {
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot1);
                    }else if (shootorder[0] == 0) {
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot2);
                    }
                    spindexer.setLowerGateOpen(true);
                    shooter.setUpperGateOpen(true);
                    spindexer.setKickerPos(true);
                    intakes.setGoodIntakePower(1);
                })
                .transitionTimed(shootWaitTime, States.ShootSpin1)

                .state(States.ShootSpin1)
                .onEnter(()->{
                    if (shootorder[1] == 2) {
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot0);
                    }else if (shootorder[1] == 1) {
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot1);
                    }else if (shootorder[1] == 0) {
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot2);
                    }
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
                    if (shootorder[2] == 2) {
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot0);
                    }else if (shootorder[2] == 1) {
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot1);
                    }else if (shootorder[2] == 0) {
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot2);
                    }
                    spindexer.setKickerPos(false);
                })
                .transitionTimed(shootWaitTime, States.Kick3)

                .state(States.Kick3)
                .onEnter(()->{
                    spindexer.setKickerPos(true);
                })
                .transitionTimed(shootWaitTime, States.BeforeIntake)
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
            telemetry.addData("Bad Intake beam break", intakes.getBadBeamBreak());
            telemetry.addData("Good Inside Intake beam break", intakes.getGoodBeamBreakInside());
            telemetry.addData("Good Outside Intake beam break", intakes.getGoodBeamBreakOutside());
            telemetry.addData("Good intake distance", intakes.getGoodIntakeDistance());
            telemetry.addData("Spindexer Position", spindexer.getCurrentPosition());

            telemetry.addData("Spindexer kick", spindexer.is_kick);
            telemetry.update();
        }
    }
}

