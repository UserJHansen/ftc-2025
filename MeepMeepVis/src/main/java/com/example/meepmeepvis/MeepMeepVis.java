package com.example.meepmeepvis;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.AddTrajectorySequenceCallback;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

import com.userjhansen.automap.AutoPart;
import com.userjhansen.automap.Maps.InsideOne;
import com.userjhansen.automap.Maps.Map;
import com.userjhansen.automap.Maps.OutsideOne;

import java.util.ArrayList;

import kotlin.Pair;

public class MeepMeepVis {

    public static void addParts(TrajectorySequenceBuilder traj, AutoPart[] parts, boolean isRed) {
//        For blue side, multiply all y values by -1 and flip the heading
        int yMult = isRed ? 1 : -1;
        int headingMult = isRed ? 1 : -1;
        for (AutoPart part : parts) {
            switch (part.type) {
                case STRAFE:
                    traj.strafeTo(part.modified(yMult, headingMult).vec());
                    break;
                case STRAFE_TO:
                    traj.lineToLinearHeading(part.modified(yMult, headingMult));
                    break;
                case TURN:
                    traj.turn(part.value);
                    break;
                case FORWARD:
                    traj.forward(part.value);
                    break;
                case BACK:
                    traj.back(part.value);
                    break;
                case WAIT:
                    traj.waitSeconds(part.value);
                    break;
                case SPLINE_TO:
                    traj.splineToSplineHeading(part.modified(yMult, headingMult), part.value);
                    break;
                case SPLINE_CONSTANT:
                    traj.splineToConstantHeading(part.modified(yMult, headingMult).vec(), 0);
                    break;
                case ACTION:
                    break;
                case CHANGE_LIGHT:
                    break;
            }
        }
    }

    public static TrajectorySequence buildTrajectorySequence(DriveShim drive, Map map, boolean isRed, boolean isInside) {
        TrajectorySequenceBuilder traj = drive.trajectorySequenceBuilder(
                new Pose2d(
                        map.getStartPosition().getX(),
                        map.getStartPosition().getY() * (isRed ? 1 : -1),
                        map.getStartPosition().getHeading() + (isRed ? 0 : Math.PI))
        );

        addParts(traj, AutoPart.makeFullAutoList(map, isInside), isRed);

        return traj.build();
    }

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        ArrayList<Pair<Boolean, AddTrajectorySequenceCallback>> trajs = new ArrayList<>();

        boolean loopIsRed = true;
        do {
            boolean isRed = loopIsRed;

            trajs.add(new Pair<>(isRed, (drive) -> buildTrajectorySequence(drive, new OutsideOne(), isRed, false)));
            trajs.add(new Pair<>(isRed, (drive) -> buildTrajectorySequence(drive, new InsideOne(), isRed, true)));

            loopIsRed = !loopIsRed;
        } while (!loopIsRed);

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f);

        for (Pair<Boolean, AddTrajectorySequenceCallback> trajBuild : trajs) {
            RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                    .setDriveTrainType(DriveTrainType.MECANUM)
                    .setColorScheme(trajBuild.getFirst() ? new ColorSchemeRedDark() : new ColorSchemeBlueDark())
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setConstraints(39.4224324932042, 39.4224324932042, Math.toRadians(143.385), Math.toRadians(163.67673913043478), 12)
                    .followTrajectorySequence(trajBuild.getSecond());
            meepMeep.addEntity(bot);
        }

        meepMeep.start();
    }
}
