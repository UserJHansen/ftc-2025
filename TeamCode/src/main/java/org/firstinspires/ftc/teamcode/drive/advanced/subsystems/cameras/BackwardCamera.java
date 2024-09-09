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
public class BackwardCamera extends VisionBase implements Camera {
    public static double cameraX = -3.7401595;
    public static double cameraY = 7.7165396;
    public static double cameraZ = -5.6692944;
    public static double firstAngle = -50;
    public static double secondAngle = 45.5904;
    public static double thirdAngle = 160;

    public String getName() {
        return "Webcam 1";
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