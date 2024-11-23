package com.example.meepmeepvis;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import com.noahbres.meepmeep.roadrunner.entity.TrajectoryActionStub;
import com.noahbres.meepmeep.roadrunner.entity.TurnActionStub;
import com.userjhansen.automap.AutoPart;
import com.userjhansen.automap.Maps.InsideOne;
import com.userjhansen.automap.Maps.Map;
import com.userjhansen.automap.Maps.OutsideOne;

public class MeepMeepVis {

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
                    traj = traj.waitSeconds(2);
                    break;
                case CHANGE_LIGHT:
                    break;
            }
        }
        return traj;
    }

    public static Action buildTrajectorySequence(DriveShim drive, Map map, boolean isRed) {
        TrajectoryActionBuilder baseTrajBuilder = drive.actionBuilder(
                new Pose2d(0,0,0)
        );

        TrajectoryActionBuilder traj = new TrajectoryActionBuilder(
                TurnActionStub::new,
                TrajectoryActionStub::new,
                new TrajectoryBuilderParams(
                        1e-6,
                        new ProfileParams(
                                0.25, 0.1, 1e-2
                        )
                ),
                map.getStartPosition(), 0.0,
                baseTrajBuilder.getBaseTurnConstraints(), baseTrajBuilder.getBaseVelConstraint(), baseTrajBuilder.getBaseAccelConstraint(),
                isRed ? pose -> pose
                        : pose -> new Pose2dDual<>(
                                pose.position.x.unaryMinus(), pose.position.y.unaryMinus(), pose.heading.plus(Math.PI))

        );

        traj = addParts(traj, AutoPart.makeFullAutoList(map));

        return traj.build();
    }

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f);

        boolean loopIsRed = true;
        do {
            boolean isRed = loopIsRed;

            RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                    .setDimensions(16.25, 17.5)
                    .setDriveTrainType(DriveTrainType.MECANUM)
                    .setColorScheme(isRed ? new ColorSchemeRedDark() : new ColorSchemeBlueDark())
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setConstraints(39.4224324932042, 39.4224324932042, Math.toRadians(143.385), Math.toRadians(163.67673913043478), 12)
                    .build();
            Action action = buildTrajectorySequence(bot.getDrive(), new OutsideOne(), isRed);
            bot.runAction(action);
            meepMeep.addEntity(bot);

            bot = new DefaultBotBuilder(meepMeep)
                    .setDimensions(16.25, 17.5)
                    .setDriveTrainType(DriveTrainType.MECANUM)
                    .setColorScheme(isRed ? new ColorSchemeRedDark() : new ColorSchemeBlueDark())
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setConstraints(55.784526, 60, Math.toRadians(143.385), Math.toRadians(163.67673913043478), 12)
                    .build();
            bot.runAction(buildTrajectorySequence(bot.getDrive(), new InsideOne(), isRed));
            meepMeep.addEntity(bot);

            loopIsRed = !loopIsRed;
        } while (!loopIsRed);

        meepMeep.start();
    }
}
