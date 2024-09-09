package org.firstinspires.ftc.teamcode.drive.advanced.subsystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class LEDStrip {
    //    Flashes per second
    public static int flashRate = 2;
    public static Deadline timeout = new Deadline(1000 / flashRate, TimeUnit.MILLISECONDS);
    final RevBlinkinLedDriver driver;
    final List<RevBlinkinLedDriver.BlinkinPattern> colours = new ArrayList<>();
    int index = 0;

    public LEDStrip(HardwareMap hardwareMap, String name) {
        this.driver = hardwareMap.get(RevBlinkinLedDriver.class, name);
    }

    public void update() {
        if (timeout.hasExpired()) {
            timeout.reset();
            if (!colours.isEmpty()) {
                if (index < colours.size() - 1) {
                    driver.setPattern(colours.get(++index));
                } else {
                    driver.setPattern(colours.get(0));
                    index = 0;
                }
            } else {
                driver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
            }
        }
    }

    public void changeTo(RevBlinkinLedDriver.BlinkinPattern pattern) {
        colours.clear();
        colours.add(pattern);
    }

    public void changeTo(List<RevBlinkinLedDriver.BlinkinPattern> patterns) {
        colours.clear();
        colours.addAll(patterns);
    }
}
