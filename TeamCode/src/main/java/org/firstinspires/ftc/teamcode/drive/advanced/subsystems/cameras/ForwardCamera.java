package org.firstinspires.ftc.teamcode.drive.advanced.subsystems.cameras;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.advanced.VisionDetection;
import org.firstinspires.ftc.teamcode.drive.advanced.subsystems.SwappableCameras;
import org.firstinspires.ftc.teamcode.drive.advanced.subsystems.VisionBase;

@Config
public class ForwardCamera extends VisionBase implements Camera {
    public static double cameraX = 0 * 0.393701;
    public static double cameraY = 0 * 0.393701;
    public static double cameraZ = 0 * 0.393701;
    public static double firstAngle = 0;
    public static double secondAngle = 0;
    public static double thirdAngle = 180;

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
    public void update(SwappableCameras cameras) {
        VisionDetection.detections = cameras.aprilProcessor.getDetections();
    }
}