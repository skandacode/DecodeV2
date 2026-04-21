package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class LimelightCamera {

    private Limelight3A limelight;


    public LimelightCamera(HardwareMap hardwareMap){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        start();
    }
    public void setCurrentPipeline(){

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
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                if (fr.getFiducialId() > 20 && fr.getFiducialId() < 24) {
                    pattern = fr.getFiducialId() - 20;
                }
            }
        }
        return pattern; //returns 0 if it doesn't see anything
    }
    public double getTrackingResults(boolean isBlueAlliance){
        double targetId = 24;
        if (isBlueAlliance){
            targetId = 20;
        }
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                if (fr.getFiducialId() == targetId) {
                    return result.getTx();
                }
            }
        }
        return 0;
    }
}
