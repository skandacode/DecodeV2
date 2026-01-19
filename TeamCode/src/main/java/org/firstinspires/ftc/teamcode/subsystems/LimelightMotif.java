package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class LimelightMotif {

    private Limelight3A limelight;

    public LimelightMotif(HardwareMap hardwareMap){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        start();
    }

    public void start(){
        limelight.start();
    }

    public void stop(){
        limelight.stop();
    }

    public int getMotif(){
        LLResult result = limelight.getLatestResult();
        int pattern = 0;
        if (result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                if (fr.getFiducialId() > 20 && fr.getFiducialId() < 24){
                    pattern = fr.getFiducialId()-20;
                }
            }
        }
        return pattern;
    }
}
