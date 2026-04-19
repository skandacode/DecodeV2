package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import solverslib.hardware.motors.Motor;

@TeleOp
@Configurable
public class TestPod extends LinearOpMode {
    Motor backRight;

    @Override
    public void runOpMode() throws InterruptedException {
        backRight = new Motor(hardwareMap, "backright");
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("position port 0 chub", backRight.getCurrentPosition());
            telemetry.update();
            backRight.update();
        }
    }
}
