package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.userjhansen.automap.AutoPart;
import com.userjhansen.automap.Maps.InsideOne;
import com.userjhansen.automap.Maps.Map;
import com.userjhansen.automap.Maps.OutsideOne;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.galahlib.actions.Loggable;
import org.firstinspires.ftc.teamcode.galahlib.actions.LoggingSequential;
import org.firstinspires.ftc.teamcode.galahlib.actions.Timeout;
import org.firstinspires.ftc.teamcode.localization.VisionDetection;
import org.firstinspires.ftc.teamcode.staticData.Logging;
import org.firstinspires.ftc.teamcode.staticData.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Auto Autonomous")
@Config
public class AutoAutonomous extends LinearOpMode {
    public static double Park_Timing = 15;

    public static TrajectoryActionBuilder addParts(TrajectoryActionBuilder traj, AutoPart[] parts) {
        for (AutoPart part : parts) {
            switch (part.type) {
                case STRAFE:
                    traj = traj.strafeTo(part.getPose().position);
                    break;
                case STRAFE_TO:
                    traj = traj.strafeToLinearHeading(part.getPose().position, part.getPose().heading);
                    break;
                case TURN:
                    traj = traj.turn(part.value);
                    break;
                case WAIT:
                    traj = traj.waitSeconds(part.value);
                    break;
                case SPLINE_TO:
                    traj = traj.splineToSplineHeading(part.getPose(), part.value);
                    break;
                case SPLINE_CONSTANT:
                    traj = traj.splineToConstantHeading(part.getPose().position, part.value);
                    break;
                case ACTION:
                    break;
                case CHANGE_LIGHT:
                    break;
            }
        }
        return traj;
    }

    @Override
    public void runOpMode() {
        MecanumDrive driveBase = new MecanumDrive(hardwareMap, PoseStorage.currentPose);
        VisionDetection visionDetection = new VisionDetection(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap);
        Logging.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Action initAction = new ParallelAction(
                intake.resetSlides(),
                outtake.resetLifts(),
                outtake.getElbow().setPosition(2),
                outtake.getWrist().setPosition(2),
                outtake.getGrabber().setPosition(1)
        );

//        Run initialisation tasks
        TelemetryPacket p = new TelemetryPacket();
        while (!isStarted() && initAction.run(p)) {
            FtcDashboard.getInstance().sendTelemetryPacket(p);
            p = new TelemetryPacket();
            Logging.update();
        }

        intake.lockout();
        outtake.lockout();

        boolean innerPosition = false;
        while (!isStarted()) {
            p = new TelemetryPacket();
            driveBase.update(p);

            PoseStorage.isRedAlliance = driveBase.localizer.currentPose.position.y < 0;

            if (gamepad1.a) innerPosition = true;
            else if (gamepad1.b) innerPosition = false;

            Logging.LOG("ALLIANCE", PoseStorage.isRedAlliance ? "RED" : "BLUE");
            Logging.LOG("POSITION", innerPosition ? "INNER" : "OUTER");

            Logging.update();
            FtcDashboard.getInstance().sendTelemetryPacket(p);
        }
        Deadline autoTimer = new Deadline(30, TimeUnit.SECONDS);

        if (isStopRequested()) return;

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        intake.unlock();
        outtake.unlock();

        Map map = innerPosition ? new InsideOne() : new OutsideOne();
        Stages state = Stages.SAMPLE_ONE_TWO;

        TrajectoryActionBuilder builder = driveBase.allianceActionBuilder(map.getStartPosition());

        driveBase.localizer.setCurrentPose(PoseStorage.isRedAlliance
                ? map.getStartPosition()
                : new Pose2d(
                -map.getStartPosition().position.x,
                -map.getStartPosition().position.y,
                map.getStartPosition().heading.plus(Math.PI).toDouble())
        );

        ActionGenerator getCapture = () -> new LoggingSequential(
                "INTAKE_CAPTURE",
                new Timeout(new Loggable("WAIT_FOR_TRANSFER", new ParallelAction(
                        intake.transfer(),
                        new SleepAction(0.75)
                )), 3.0),
                intake.stopTransfer(),
                outtake.pickupInternalSample()
        );

        builder = builder.strafeTo(map.getSpecimenPosition().position)
                .afterDisp(0, outtake.grabber(false))
                .stopAndAdd(outtake.placeSpecimen());

        builder = addParts(builder, map.getIntakeParts()[0])
                .afterTime(1, new ParallelAction(intake.captureSample(true),
                        outtake.homePosition()))
                .stopAndAdd(getCapture.run());
        builder = addParts(builder, map.getDepositParts())
                .afterTime(0, outtake.topBasket()).stopAndAdd(outtake.dropSample());

        builder = addParts(builder, map.getIntakeParts()[1])
                .afterTime(1, new ParallelAction(intake.captureSample(true),
                        new SequentialAction(outtake.retractArm(), outtake.homePosition())))
                .stopAndAdd(getCapture.run());
        builder = addParts(builder, map.getDepositParts())
                .afterTime(0, outtake.topBasket()).stopAndAdd(outtake.dropSample());

        Action autonomous = builder.build();

        while (opModeIsActive() && !isStopRequested()) {
            p = new TelemetryPacket();

            driveBase.update(p);
            visionDetection.update(driveBase.localizer, p);
            PoseStorage.currentPose = driveBase.localizer.currentPose;

            if (!autonomous.run(p)) {
                switch (state) {
                    case SAMPLE_ONE_TWO:
//                        Check if there is time to do the third sample
                        if (autoTimer.timeRemaining(TimeUnit.SECONDS) > Park_Timing) {
//                            Can do the third sample
                            builder = builder.fresh();
                            builder = addParts(builder, map.getIntakeParts()[2])
                                    .afterTime(1, new ParallelAction(intake.captureSample(true),
                                            outtake.homePosition()))
                                    .stopAndAdd(getCapture.run());
                            builder = addParts(builder, map.getDepositParts())
                                    .afterTime(0, outtake.topBasket()).stopAndAdd(outtake.dropSample());


                            autonomous = builder.build();

                            state = Stages.OPTIONAL_THREE;
                            break;
                        }
                        state = Stages.PARK;
                    case OPTIONAL_THREE:
//                        Time to park

                        builder = builder.fresh();

                        builder = addParts(builder, map.getParkParts())
                                .afterTime(0, outtake.retractArm());

                        autonomous = builder.build();
                        break;
                    case PARK:
//                        Finished parking, we're done here
                        return;

                }
            }

            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }
            driveBase.logState("[AUTO]");
            intake.logState("[AUTO]");
            outtake.logState("[AUTO]");
            Logging.update();
            FtcDashboard.getInstance().sendTelemetryPacket(p);
        }

    }

    enum Stages {
        SAMPLE_ONE_TWO,
        OPTIONAL_THREE,
        PARK
    }

    interface ActionGenerator {
        Action run();
    }
}