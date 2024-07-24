package org.firstinspires.ftc.teamcode.drive.advanced;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.userjhansen.automap.AutoPart;
import com.userjhansen.automap.Maps.InsideOne;
import com.userjhansen.automap.Maps.InsideThree;
import com.userjhansen.automap.Maps.InsideTwo;
import com.userjhansen.automap.Maps.Map;
import com.userjhansen.automap.Maps.OutsideOne;
import com.userjhansen.automap.Maps.OutsideThree;
import com.userjhansen.automap.Maps.OutsideTwo;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.drive.AutoRunner;
import org.firstinspires.ftc.teamcode.drive.advanced.subsystems.ArmTarget;
import org.firstinspires.ftc.teamcode.drive.advanced.subsystems.Logging;

import java.util.concurrent.TimeUnit;

@Autonomous(group = "advanced", name = "0 Autonomous")
@Config
public class AutoOpmode extends LinearOpMode {
    AutoRunner autoRunner;


    void waitForPath() {
        while (opModeIsActive() && !isStopRequested() && autoRunner.driveBase.isBusy()) {
            autoRunner.update();
        }
    }

    void waitForTimeout(int milliseconds) {
        Deadline timeout = new Deadline(milliseconds, TimeUnit.MILLISECONDS);
        while (opModeIsActive() && !isStopRequested() && !timeout.hasExpired()) {
            autoRunner.update();
        }
    }

    public static double INSIDE_CUTOFF = -9;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        autoRunner = new AutoRunner(hardwareMap, telemetry);
//        autoRunner.camera.selectCameraTwo(false);
        autoRunner.liftArmAssembly.target = ArmTarget.Primed;
        autoRunner.liftArmAssembly.update(true, true);

        while (!isStarted()) {
            autoRunner.updateInit();
            Logging.LOG("DETECTED_POSITION", VisionDetection.position);
        }
//        autoRunner.camera.selectCameraTwo(false);

        if (isStopRequested()) return;

        boolean isRed = autoRunner.driveBase.getPoseEstimate().getY() < 0;
        Map autoMap = new InsideOne();
        if (autoRunner.driveBase.getPoseEstimate().getX() > INSIDE_CUTOFF) {
            switch (VisionDetection.position) {
                case 1:
                    autoMap = new InsideOne();
                    break;
                case 2:
                    autoMap = new InsideTwo();
                    break;
                case 3:
                    autoMap = new InsideThree();
                    break;
            }
        } else {
            switch (VisionDetection.position) {
                case 1:
                    autoMap = new OutsideOne();
                    break;
                case 2:
                    autoMap = new OutsideTwo();
                    break;
                case 3:
                    autoMap = new OutsideThree();
                    break;
            }
        }

        int yMult = isRed ? 1 : -1;
        int headingMult = isRed ? 1 : -1;
//        autoRunner.driveBase.setPoseEstimate(new Pose2d(
//                autoMap.getStartPosition().getX(),
//                autoMap.getStartPosition().getY()*(isRed ? 1 : -1),
//                autoMap.getStartPosition().getHeading()+(isRed ? 0 : Math.PI)));

        AutoPart[] parts = AutoPart.makeFullAutoList(autoMap, autoRunner.driveBase.getPoseEstimate().getX() > INSIDE_CUTOFF);

        RevBlinkinLedDriver.BlinkinPattern[] lightPatterns = {
                RevBlinkinLedDriver.BlinkinPattern.RED,
                RevBlinkinLedDriver.BlinkinPattern.ORANGE,
                RevBlinkinLedDriver.BlinkinPattern.GREEN
        };

        Deadline timeout;
        for (AutoPart part : parts) {
            try {
                switch (part.type) {
                    case STRAFE:
                        autoRunner.driveBase.followTrajectoryAsync(
                                autoRunner.driveBase.trajectoryBuilder(
                                                autoRunner.driveBase.getPoseEstimate()
                                        )
                                        .strafeTo(part.modified(yMult, headingMult).vec())
                                        .build()
                        );
                        break;
                    case STRAFE_TO:
                        autoRunner.driveBase.followTrajectoryAsync(
                                autoRunner.driveBase.trajectoryBuilder(
                                                autoRunner.driveBase.getPoseEstimate()
                                        )
                                        .lineToLinearHeading(part.modified(yMult, headingMult))
                                        .build()
                        );
                        break;
                    case TURN:
                        autoRunner.driveBase.turnAsync(part.value);
                        break;
                    case FORWARD:
                        autoRunner.driveBase.followTrajectoryAsync(
                                autoRunner.driveBase.trajectoryBuilder(
                                                autoRunner.driveBase.getPoseEstimate()
                                        ).forward(part.value)
                                        .build()
                        );
                        break;
                    case BACK:
                        autoRunner.driveBase.followTrajectoryAsync(
                                autoRunner.driveBase.trajectoryBuilder(
                                                autoRunner.driveBase.getPoseEstimate()
                                        )
                                        .back(part.value)
                                        .build()
                        );
                        break;
                    case WAIT:
                        timeout = new Deadline((long) part.value, java.util.concurrent.TimeUnit.SECONDS);
                        while (opModeIsActive() && !isStopRequested() && !timeout.hasExpired()) {
                            autoRunner.update();
                        }
                        break;
                    case SPLINE_TO:
                        autoRunner.driveBase.followTrajectoryAsync(
                                autoRunner.driveBase.trajectoryBuilder(
                                                autoRunner.driveBase.getPoseEstimate()
                                        )
                                        .splineToSplineHeading(part.modified(yMult, headingMult), part.value)
                                        .build()
                        );
                        break;
                    case SPLINE_CONSTANT:
                        autoRunner.driveBase.followTrajectoryAsync(
                                autoRunner.driveBase.trajectoryBuilder(
                                                autoRunner.driveBase.getPoseEstimate()
                                        )
                                        .splineToConstantHeading(part.modified(yMult, headingMult).vec(), 0)
                                        .build()
                        );
                        break;
                    case CHANGE_LIGHT:
                        autoRunner.currentPattern = lightPatterns[(int) part.value];
                        break;
                    case ACTION:
                        switch ((int) part.value) {
                            case 0:
                                autoRunner.intake.changeTo(2);
                                waitForTimeout(500);
                                autoRunner.driveBase.followTrajectoryAsync(
                                        autoRunner.driveBase.trajectoryBuilder(
                                                autoRunner.driveBase.getPoseEstimate()
                                        ).back(2).build()
                                );
                                waitForPath();
                                waitForTimeout(500);
                                autoRunner.intake.changeTo(false);
                                break;
                            case 1:
                                autoRunner.liftArmAssembly.target = ArmTarget.LiftRaised;
                                autoRunner.liftArmAssembly.liftTargetHeight = 15;

                                waitForTimeout(2000);

                                autoRunner.liftArmAssembly.target = ArmTarget.Drop;
                                waitForTimeout(1000);
                                break;
                        }
                        break;
                }
            } catch (Exception e) {
                continue;
            }

            waitForPath();

            if (!opModeIsActive() || isStopRequested()) {
                return;
            }
        }

        PoseStorage.currentPose = autoRunner.driveBase.getPoseEstimate();
    }
}