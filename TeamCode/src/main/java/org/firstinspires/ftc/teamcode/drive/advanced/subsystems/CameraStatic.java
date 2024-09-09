package org.firstinspires.ftc.teamcode.drive.advanced.subsystems;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
                            .addTags(VisionBase.getFixedTags())
                            .build())
            .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES);
}
