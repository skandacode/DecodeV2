package org.firstinspires.ftc.teamcode.Logging;

import android.os.Environment;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

public class FileLogger {
    private static final String FOLDER_PATH = Environment.getExternalStorageDirectory().getPath() + "/FIRST/logs/";
    private static String currentFileName = "log.txt";

    public static void startNewLog() {
        File directory = new File(FOLDER_PATH);
        if (!directory.exists()) {
            boolean success = directory.mkdirs();
        }
        
        String timeStamp = new SimpleDateFormat("yyyy-MM-dd_HH-mm-ss", Locale.US).format(new Date());
        currentFileName = "Log_" + timeStamp + ".txt";
    }

    public static void writeLog(String tag, String message) {
        String timestamp = new SimpleDateFormat("HH:mm:ss.SSS", Locale.US).format(new Date());
        String logData = timestamp + " [" + tag + "]: " + message + "\n";

        try {
            File file = new File(FOLDER_PATH + currentFileName);
            FileWriter writer = new FileWriter(file, true); // true for append mode
            writer.write(logData);
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public static void writeLog(String message) {
        writeLog("INFO", message);
    }
}
