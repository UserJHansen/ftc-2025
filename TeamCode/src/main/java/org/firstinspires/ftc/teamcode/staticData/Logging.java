package org.firstinspires.ftc.teamcode.staticData;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.messages.ColorMessage;
import org.firstinspires.ftc.teamcode.messages.DoubleMessage;
import org.firstinspires.ftc.teamcode.messages.StringMessage;

@Config
public class Logging {
    public static boolean DEBUG = true;

    public static Telemetry telemetry;
    static ElapsedTime timer = new ElapsedTime();

    public static <T> void DEBUG(String caption, T data) {
        if (data.getClass() == String.class) {
            FlightRecorder.write(caption, new StringMessage((String) data));
        } else if (data.getClass() == Double.class) {
            FlightRecorder.write(caption, new DoubleMessage((Double) data));
        } else if (data.getClass() == ColorMessage.class) {
            FlightRecorder.write(caption, data);
        }

        if (DEBUG) {
            telemetry.addData(caption, data);
        }
    }

    public static void DEBUG(String caption) {
        FlightRecorder.write(caption, System.nanoTime());
        if (DEBUG) {
            telemetry.addLine(caption);
        }
    }

    public static <T> void LOG(String caption, T data) {
        if (data.getClass() == String.class) {
            FlightRecorder.write(caption, new StringMessage((String) data));
        } else if (data.getClass() == Double.class) {
            FlightRecorder.write(caption, new DoubleMessage((Double) data));
        } else if (data.getClass() == ColorMessage.class) {
            FlightRecorder.write(caption, data);
        }

        telemetry.addData(caption, data);
    }

    public static void LOG(String caption) {
        FlightRecorder.write(caption, System.nanoTime());
        telemetry.addLine(caption);
    }

    public static void update() {
        double elapsedTime = timer.milliseconds();
        timer.reset();
        Logging.LOG("CPS", 1000 / elapsedTime);

        telemetry.update();
    }
}
