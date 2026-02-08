package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

public class PositionLogger {
    private BufferedWriter writer;
    private SimpleDateFormat timeFormat;

    public PositionLogger(String filename) throws IOException {
        writer = new BufferedWriter(new FileWriter(filename, true));
        timeFormat = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS");
        writer.write("=== Position Log Started at " + timeFormat.format(new Date()) + " ===\n");
        writer.flush();
    }

    public void logPose(Pose pose) throws IOException {
        String timestamp = timeFormat.format(new Date());
        String logEntry = String.format(Locale.US, "[%s] X: %.3f, Y: %.3f, Heading: %.3f°\n",
                timestamp, pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
        writer.write(logEntry);
        writer.flush();
    }

    public void close() throws IOException {
        if (writer != null) {
            writer.write("=== Position Log Ended at " + timeFormat.format(new Date()) + " ===\n");
            writer.close();
        }
    }
}
