package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.bylazar.configurables.annotations.Configurable;


@Configurable
public class LLFieldScannerResults {
    public final double tx, ty;

    public static double cameraHeight = 11;

    public LLFieldScannerResults(double tx, double ty) {
        this.tx = tx;
        this.ty = ty;
    }

    @NonNull
    @Override
    public String toString(){
        return "FieldScannerResult<" + tx + ", " + ty + ">";
    }


    public double[] getPosition(){
        double tyRadians = Math.toRadians(ty);
        double txRadians = Math.toRadians(tx);

        double forwardDistance =  - cameraHeight / Math.tan(tyRadians);
        double sidewaysDistance = forwardDistance * Math.tan(txRadians);

        return new double[]{forwardDistance, sidewaysDistance};
    }
}
