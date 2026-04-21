package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class LimelightCamera {

    private Limelight3A limelight;

    public enum Pipelines{
        MOTIF (0),
        BLUETRACK (1),
        REDTRACK (2);

        public final int index;

        Pipelines(int index) {
            this.index = index;
        }
    }

    private Pipelines currentPipeline = Pipelines.MOTIF;

    public LimelightCamera(HardwareMap hardwareMap){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(currentPipeline.index);
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

    public void setCurrentPipeline(Pipelines pipeline){
        limelight.pipelineSwitch(pipeline.index);
        currentPipeline = pipeline;
    }

    public int getMotif(){
        if (currentPipeline == Pipelines.MOTIF) {
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
        }else{
            return -1;
        }
    }
    public double getTrackingResults(){
        if (currentPipeline == Pipelines.MOTIF){
            return 0;
        }
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            return result.getTx();
        }
        return 0;
    }

    public Pipelines getCurrentPipeline(){
        return currentPipeline;
    }
}
