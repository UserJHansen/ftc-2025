package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.staticData.Logging;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.List;

@TeleOp(group = "testing", name = "Reaction Speed Test")
@Disabled
public class ReactionSpeedTest extends LinearOpMode {
    public Intake intake;

    @Override
    public void runOpMode() {
        intake = new Intake(hardwareMap);

        Logging.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Action initAction = new ParallelAction(
                intake.resetSlides()
        );

//        Run initialisation tasks
        TelemetryPacket p = new TelemetryPacket();
        while (!isStarted() && initAction.run(p)) {
            FtcDashboard.getInstance().sendTelemetryPacket(p);
            p = new TelemetryPacket();
            Logging.update();
        }

        while (!isStarted()) {
            sleep(10);
        }

        if (isStopRequested()) return;

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        while (opModeIsActive() && !isStopRequested()) {
            Action action = intake.captureSample(true);

            p = new TelemetryPacket();
            while (action.run(p)) {
                Logging.update();
                for (LynxModule module : allHubs) {
                    module.clearBulkCache();
                }
                FtcDashboard.getInstance().sendTelemetryPacket(p);
                p = new TelemetryPacket();
            }
        }
    }
}