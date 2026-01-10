package solverslib.hardware;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class CachedColorSensorV3 extends RevColorSensorV3 {
    private long last_updated_ms = 0;
    private int interval_read_ms=200;
    private double cached_distance = 0.0;

    public CachedColorSensorV3(I2cDeviceSynchSimple deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);
    }

    public double getDistance(DistanceUnit distanceUnit){
        return this.getDistance(distanceUnit, false);
    }

    public double getDistance(DistanceUnit distanceUnit, boolean forceRefresh){
        if (System.currentTimeMillis()-last_updated_ms>interval_read_ms || forceRefresh){
            cached_distance = super.getDistance(distanceUnit);
            last_updated_ms = System.currentTimeMillis();
        }
        return cached_distance;
    }

    public int getInterval_read_ms() {
        return interval_read_ms;
    }

    public void setInterval_read_ms(int interval_read_ms) {
        this.interval_read_ms = interval_read_ms;
    }

    public long getLastRefresh(){
        return last_updated_ms;
    }
}
