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
        BALLTRACKING (1);

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
    public LLFieldScannerResults getTrackingResults(){
        if (currentPipeline == Pipelines.BALLTRACKING){
            limelight.updatePythonInputs(new double[] {0, 0, 0, 0, 0, 0, 0, 0});
            LLResult result = limelight.getLatestResult();
            if (result != null) {
                System.out.println(result.getTx()+"    "+result.getTy());
                double[] output = result.getPythonOutput();
                if (output == null){
                    System.out.println("Output is null");
                    return null;
                }
                if (output.length == 0) {
                    System.out.println("length is 0");
                    return null;
                }
                if (output[1] == 1.0) {
                    return new LLFieldScannerResults(result.getTx(), result.getTy());
                }else{
                    return null;
                }
            }
            System.out.println("Not valid");
        }
        System.out.println("returning null");
        return null;
    }

    public Pipelines getCurrentPipeline(){
        return currentPipeline;
    }
}
