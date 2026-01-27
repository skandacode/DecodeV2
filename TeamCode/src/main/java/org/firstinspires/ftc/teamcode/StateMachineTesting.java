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
    public int pattern = 1;
    public static boolean shooterButton = false;
    public static double shootWaitTime = 0.25;
    public static boolean rapidFire = true;

    public enum States{
        Intake,
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
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs)
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        intakes = new Intakes(hardwareMap);
        shooter = new Shooter(hardwareMap);
        spindexer = new Spindexer(hardwareMap);

        while (opModeInInit()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();
            telemetry.update();
        }

        waitForStart();

        StateMachine stateMachine = new StateMachineBuilder()
                .state(States.Intake)
                .loop(()->{
                    shooter.setUpperGateOpen(false);
                    spindexer.setLowerGateOpen(rapidFire);
                    spindexer.setKickerPos(false);

                    if (rapidFire) {
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot1);

                        intakes.setGoodIntakePower(1);
                        intakes.setBadIntakePower(-0.1);
                    }else{
                        spindexer.setPosition(Spindexer.SpindexerPosition.Intake1);
                        intakes.setGoodIntakePower(0.2);
                        intakes.setBadIntakePower(1);
                    }
                })
                .transition(()->shooterButton && rapidFire, States.OpenUpperGate)
                .transition(()->shooterButton && !rapidFire, States.Kick1)
                .transition(()->intakes.getBadIntakeDetected() && !rapidFire, States.Wait1)

                .state(States.Wait1)
                .onEnter(()->{
                    spindexer.setPosition(Spindexer.SpindexerPosition.Intake2);
                })
                .transitionTimed(shootWaitTime, States.Increment2)
                .transition(()->shooterButton, States.Kick1)

                .state(States.Increment2)
                .onEnter(()->{
                    spindexer.setPosition(Spindexer.SpindexerPosition.Intake2);
                })
                .transition(()->intakes.getBadIntakeDetected(), States.Wait2)
                .transition(()->shooterButton, States.Kick1)

                .state(States.Wait2)
                .onEnter(()->{
                    spindexer.setPosition(Spindexer.SpindexerPosition.Intake3);
                })
                .transitionTimed(shootWaitTime, States.Increment3)
                .transition(()->shooterButton, States.Kick1)

                .state(States.Increment3)
                .onEnter(()->{
                    spindexer.setPosition(Spindexer.SpindexerPosition.Intake3);
                })
                .transition(()->intakes.getBadIntakeDetected(), States.WaitForShoot)
                .transition(()->shooterButton, States.Kick1)

                .state(States.WaitForShoot)
                .loop(()->{
                    if (shootorder[0]==0){
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot1);
                    }else if (shootorder[0]==1){
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot2);
                    }else{
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot3);
                    }
                    intakes.setBadIntakePower(0.3);
                    intakes.setGoodIntakePower(1);

                    shooter.setUpperGateOpen(true);
                })
                .transition(()->shooterButton, States.Kick1)

                .state(States.Kick1)
                .onEnter(()->{
                    shooterButton=false;
                    if (shootorder[0]==0){
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot1);
                    }else if (shootorder[0]==1){
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot2);
                    }else{
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot3);
                    }
                    spindexer.setLowerGateOpen(true);
                    shooter.setUpperGateOpen(true);
                    spindexer.setKickerPos(true);
                })
                .transitionTimed(shootWaitTime, States.ShootSpin1)

                .state(States.ShootSpin1)
                .onEnter(()->{
                    if (shootorder[1]==0){
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot1);
                    }else if (shootorder[1]==1){
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot2);
                    }else{
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot3);
                    }
                })
                .transitionTimed(shootWaitTime, States.ShootSpin2)

                .state(States.ShootSpin2)
                .onEnter(()->{
                    if (shootorder[2]==0){
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot1);
                    }else if (shootorder[2]==1){
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot2);
                    }else{
                        spindexer.setPosition(Spindexer.SpindexerPosition.Shoot3);
                    }
                })
                .transitionTimed(shootWaitTime, States.Intake)

                .state(States.OpenUpperGate)
                .onEnter(()->{
                    shooterButton=false;
                    spindexer.setLowerGateOpen(true);
                    shooter.setUpperGateOpen(true);
                })
                .transitionTimed(0.2, States.Shoot)

                .state(States.Shoot)
                .onEnter(()->{
                    spindexer.setKickerPos(true);
                })
                .transitionTimed(0.5, States.Intake)
                .build();



        stateMachine.start();

        while (opModeIsActive()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();
            stateMachine.update();
            intakes.update();
            shooter.update();
            spindexer.update();
            telemetry.addData("State: ", stateMachine.getState());
            telemetry.addData("Shooter Target", shooter.getTargetVelo());
            telemetry.addData("Shooter Velocity", shooter.getCurrentVelocity());
            telemetry.addData("Spindexer kick", spindexer.is_kick);
            telemetry.update();
        }
    }
}

