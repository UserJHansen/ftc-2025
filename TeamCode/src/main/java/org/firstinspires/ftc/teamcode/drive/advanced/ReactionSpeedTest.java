package org.firstinspires.ftc.teamcode.drive.advanced;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.advanced.subsystems.Logging;
import org.firstinspires.ftc.teamcode.drive.advanced.subsystems.SelectiveIntakeMotor;

import java.util.List;

@TeleOp(group = "testing", name = "Reaction Speed Test")
public class ReactionSpeedTest extends LinearOpMode {
    public SelectiveIntakeMotor motor;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = new SelectiveIntakeMotor(hardwareMap);

        Logging.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        while (!isStarted()) {
            sleep(10);
        }

        if (isStopRequested()) return;

        motor.startSeeking();

        while (opModeIsActive() && !isStopRequested()) {
            motor.update();

            Logging.update();
            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }
        }
    }
}