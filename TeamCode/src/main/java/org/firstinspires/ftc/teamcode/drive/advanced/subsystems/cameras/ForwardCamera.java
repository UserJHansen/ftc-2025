package org.firstinspires.ftc.teamcode.drive.advanced.subsystems.cameras;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.advanced.VisionDetection;
import org.firstinspires.ftc.teamcode.drive.advanced.subsystems.CameraStatic;
import org.firstinspires.ftc.teamcode.drive.advanced.subsystems.Logging;
import org.firstinspires.ftc.teamcode.drive.advanced.subsystems.VisionBase;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Config
public class ForwardCamera extends VisionBase implements Camera {
    public static double cameraX = 0;
    public static double cameraY = 0;
    public static double cameraZ = 0;
    public static double firstAngle = 0;
    public static double secondAngle = 0;
    public static double thirdAngle = 0;

    public VisionPortal visionPortal;
    public AprilTagProcessor aprilProcessor;

    ForwardCamera(HardwareMap hardwareMap) {
        aprilProcessor = CameraStatic.baseAprilTags
                .setDrawAxes(Logging.DEBUG)
                .setDrawCubeProjection(Logging.DEBUG)
                .setDrawTagOutline(Logging.DEBUG)

//                These have been calculated from the data we collected
                .setLensIntrinsics(1485.792154, 1475.793189, 911.2968158, 513.6169402)
                .build();

        WebcamName webcam = hardwareMap.get(WebcamName.class, this.getName());

        visionPortal = CameraStatic.baseVisionPortal
                .setCamera(webcam)
                .setCameraResolution(new Size(1920, 1080)) // High quality for longer viewing

                .addProcessor(aprilProcessor)
                .build();
    }

    public String getName() {
        return "Webcam 2";
    }

    public OpenGLMatrix getCameraPos() {
        return OpenGLMatrix.identityMatrix()
                .translated((float) cameraX, (float) cameraY, (float) cameraZ)
                .multiplied(new Orientation(AxesReference.INTRINSIC, AxesOrder.XYZ,
                        AngleUnit.DEGREES, (float) firstAngle, (float) secondAngle, (float) thirdAngle, 0)
                        .getRotationMatrix());
    }

    @Override
    public void update() {
        VisionDetection.detections = aprilProcessor.getDetections();
    }
}