package com.example.meepmeepvis;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import com.userjhansen.automap.AutoPart;
import com.userjhansen.automap.Maps.InsideOne;
import com.userjhansen.automap.Maps.Map;
import com.userjhansen.automap.Maps.OutsideOne;

import java.util.ArrayList;

import kotlin.Pair;

public class MeepMeepVis {

    public static TrajectoryActionBuilder addParts(TrajectoryActionBuilder traj, AutoPart[] parts, boolean isRed) {
//        For blue side, multiply all y values by -1 and flip the heading
        int yMult = isRed ? 1 : -1;
        int headingMult = isRed ? 1 : -1;
        for (AutoPart part : parts) {
            switch (part.type) {
                case STRAFE:
                    traj = traj.strafeTo(part.modified(yMult, headingMult).position);
                    break;
                case STRAFE_TO:
                    traj = traj.strafeToLinearHeading(part.modified(yMult, headingMult).position, part.modified(yMult, headingMult).heading);
                    break;
                case TURN:
                    traj = traj.turn(part.value);
                    break;
                case WAIT:
                    traj = traj.waitSeconds(part.value);
                    break;
                case SPLINE_TO:
                    traj = traj.splineToSplineHeading(part.modified(yMult, headingMult), part.value * headingMult);
                    break;
                case SPLINE_CONSTANT:
                    traj = traj.splineToConstantHeading(part.modified(yMult, headingMult).position, 0);
                    break;
                case ACTION:
                    traj = traj.waitSeconds(5);
                    break;
                case CHANGE_LIGHT:
                    break;
            }
        }
        return traj;
    }

    public static Action buildTrajectorySequence(DriveShim drive, Map map, boolean isRed) {
        TrajectoryActionBuilder traj = drive.actionBuilder(
                new Pose2d(
                        map.getStartPosition().position.x,
                        map.getStartPosition().position.y * (isRed ? 1 : -1),
                        map.getStartPosition().heading.real + (isRed ? 0 : Math.PI))
        );

        traj = addParts(traj, AutoPart.makeFullAutoList(map), isRed);

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
                    .setDriveTrainType(DriveTrainType.MECANUM)
                    .setColorScheme(isRed ? new ColorSchemeRedDark() : new ColorSchemeBlueDark())
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setConstraints(39.4224324932042, 39.4224324932042, Math.toRadians(143.385), Math.toRadians(163.67673913043478), 12)
                    .build();
            Action action = buildTrajectorySequence(bot.getDrive(), new OutsideOne(), isRed);
            bot.runAction(action);
            meepMeep.addEntity(bot);

            bot = new DefaultBotBuilder(meepMeep)
                    .setDriveTrainType(DriveTrainType.MECANUM)
                    .setColorScheme(isRed ? new ColorSchemeRedDark() : new ColorSchemeBlueDark())
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setConstraints(39.4224324932042, 39.4224324932042, Math.toRadians(143.385), Math.toRadians(163.67673913043478), 12)
                    .build();
            bot.runAction(buildTrajectorySequence(bot.getDrive(), new InsideOne(), isRed));
            meepMeep.addEntity(bot);

            loopIsRed = !loopIsRed;
        } while (!loopIsRed);

        meepMeep.start();
    }
}
