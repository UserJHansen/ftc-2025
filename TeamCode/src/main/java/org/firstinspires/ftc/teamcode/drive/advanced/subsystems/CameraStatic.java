package org.firstinspires.ftc.teamcode.drive.advanced.subsystems;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class CameraStatic {
    public static VisionPortal.Builder baseVisionPortal = new VisionPortal.Builder()
            .enableLiveView(true)
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
            .setAutoStopLiveView(false);

    public static AprilTagProcessor.Builder baseAprilTags = new AprilTagProcessor.Builder()
//                Tags
            .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
            .setTagLibrary(
                    new AprilTagLibrary.Builder()
//                          Special custom tag for the initial randomisation detection
                            .addTag(19, "CustomDetector",
                                    2.303, new VectorF(0, 0, 0), DistanceUnit.INCH,
                                    new Quaternion(0, 0, 0, 0, 0))

//                            THE FTC LIB TAGS ARE BROKEN this is why we don't use them
//                            .addTags(AprilTagGameDatabase.getCenterStageTagLibrary())
                            .addTags(VisionBase.getFixedTags())

                            .build())
            .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES);
}
