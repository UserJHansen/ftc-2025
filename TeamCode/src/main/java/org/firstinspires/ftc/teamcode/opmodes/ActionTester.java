package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.galahlib.actions.race;
import org.firstinspires.ftc.teamcode.staticData.Logging;
import org.firstinspires.ftc.teamcode.staticData.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

import java.util.List;

@Autonomous(group = "advanced", name = "Action Tester")
public class ActionTester extends LinearOpMode {
    @Override
    public void runOpMode() {
        MecanumDrive driveBase = new MecanumDrive(hardwareMap, PoseStorage.currentPose);
        Intake intake = new Intake(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap);

        Logging.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        intake.lockout();
        outtake.lockout();

        TelemetryPacket p;
        while (!isStarted()) {
            p = new TelemetryPacket();
            driveBase.update(p);
            Logging.update();
            FtcDashboard.getInstance().sendTelemetryPacket(p);
        }

        if (isStopRequested()) return;

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        Action action = null;
        while (opModeIsActive() && !isStopRequested()) {
            p = new TelemetryPacket();

            PoseStorage.shouldHallucinate = (PoseStorage.splitControls ? gamepad2 : gamepad1).guide;
            if (gamepad1.dpad_right) {
                PoseStorage.isRedAlliance = true;
            } else if (gamepad1.dpad_left) {
                PoseStorage.isRedAlliance = false;
            }

            driveBase.update(p);

            if (action == null || !action.run(p)) {
//                action = new SequentialAction(
//                        intake.captureSample(true),
//                        new race(intake.transfer(),
//                                new SleepAction(2)
//                        ),
//                        intake.stopTransfer()
//                );
                action = outtake.getLift().gotoDistance(10);
//                action = new SequentialAction(
//                    outtake.readyForTransfer()
//                );
            }

            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }
            driveBase.logState("[TELEOP]");
            intake.logState("[TELEOP]");
            outtake.logState("[TELEOP]");
            Logging.update();
            FtcDashboard.getInstance().sendTelemetryPacket(p);
        }
    }
}