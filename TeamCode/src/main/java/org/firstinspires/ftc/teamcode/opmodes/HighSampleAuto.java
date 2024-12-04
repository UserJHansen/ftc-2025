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
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.userjhansen.automap.AutoPart;
import com.userjhansen.automap.Maps.InsideOneSample;
import com.userjhansen.automap.Maps.Map;
import com.userjhansen.automap.Maps.OutsideOneSample;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.localization.VisionDetection;
import org.firstinspires.ftc.teamcode.staticData.Logging;
import org.firstinspires.ftc.teamcode.staticData.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

import java.util.List;

@Autonomous(name = "High Sample Auto")
@Config
public class HighSampleAuto extends LinearOpMode {
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

    public static TrajectoryActionBuilder addActionAfterFirst(TrajectoryActionBuilder traj, AutoPart[] parts, Action action) {
        boolean addedFirst = false;
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

            if (!addedFirst) {
                addedFirst = true;
                traj = traj.afterTime(0.1, action);
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
                new SequentialAction(
                        outtake.resetLifts(),
                        outtake.getLift().gotoDistance(1.0),
                        outtake.specimenBack(),
                        outtake.grabber(true),
                        new SleepAction(1),
                        outtake.grabber(false)
                )
        );

//        Run initialisation tasks
        outtake.unlock();
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

            if (gamepad1.a) innerPosition = true;
            else if (gamepad1.y) innerPosition = false;

            if (gamepad1.b) {
                PoseStorage.isRedAlliance = true;
            } else if (gamepad1.x) {
                PoseStorage.isRedAlliance = false;

            }

            Logging.LOG("ALLIANCE", PoseStorage.isRedAlliance ? "RED" : "BLUE");
            Logging.LOG("POSITION", innerPosition ? "INNER" : "OUTER");

            Logging.update();
            FtcDashboard.getInstance().sendTelemetryPacket(p);
        }

        if (isStopRequested()) return;

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        intake.unlock();
        outtake.unlock();

        Map map = innerPosition ? new InsideOneSample() : new OutsideOneSample();

        TrajectoryActionBuilder builder = driveBase.allianceActionBuilder(map.getStartPosition());

        driveBase.localizer.setCurrentPose(PoseStorage.isRedAlliance
                ? map.getStartPosition()
                : new Pose2d(
                -map.getStartPosition().position.x,
                -map.getStartPosition().position.y,
                map.getStartPosition().heading.plus(Math.PI).toDouble()));

        builder = builder.strafeTo(map.getHighBasketPosition2().position)
                .stopAndAdd(
                        new SequentialAction(
                                outtake.topBasket()
                        )
                );

        builder = builder.strafeTo(map.getHighBasketPosition().position)
                .stopAndAdd(
                        new SequentialAction(
                                outtake.dropSample()
                        )
                );

//        builder = addParts(builder, map.getSpecimenParts());
//
        builder = builder.strafeTo(map.getHighBasketPosition2().position)
                .stopAndAdd(
                        new SequentialAction(
                                outtake.retractArm(),
                                outtake.homePosition()
                        )
                );

        builder = addParts(builder, map.getParkParts());
//
//        builder = builder.strafeTo(map.getSampleCollectPosition().position)
//                .stopAndAdd(
//                        new SequentialAction(
//                                intake.retractSlides(),
//                                intake.transfer(),
//                                outtake.pickupInternalSample()
//                        )
//                ).strafeTo(new Vector2d(0, -52));
//
//        builder = addParts(builder, map.getDepositParts());
//
//        builder = builder.strafeTo(map.getHighBasketPosition().position)
//                .stopAndAdd(
//                        new SequentialAction(
//                                outtake.topBasket(),
//                                outtake.dropSample()
//                        )
//                ).strafeTo(new Vector2d(0, -52));
//
//        builder = addParts(builder, map.getSpecimenParts());
//
//        builder = builder.strafeTo(map.getParkPosition().position)
//                .stopAndAdd(
//                        new SequentialAction(
//                                outtake.retractArm(),
//                                outtake.homePosition()
//                        )
//                ).strafeTo(new Vector2d(0, -52));
//
//        builder = addParts(builder, map.getParkParts());

        Action autonomous = builder.build();


        while (opModeIsActive() && !isStopRequested() && autonomous.run(p)) {
            p = new TelemetryPacket();

            driveBase.update(p);
//            visionDetection.update(driveBase.localizer, p);
            PoseStorage.currentPose = driveBase.localizer.currentPose;

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
}