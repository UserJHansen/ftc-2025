package org.firstinspires.ftc.teamcode.drive.advanced.subsystems;


import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Logging {
    public static boolean DEBUG = false;

    public static Telemetry telemetry;

    public static void DEBUG(String caption, Object data) {
        if (DEBUG) {
            telemetry.addData(caption, data);
        }
    }

    public static void DEBUG(String caption) {
        if (DEBUG) {
            telemetry.addLine(caption);
        }
    }

    public static void LOG(String caption, Object data) {
        telemetry.addData(caption, data);
    }

    public static void LOG(String caption) {
        telemetry.addLine(caption);
    }

    public static void update() {
        telemetry.update();
    }
}
