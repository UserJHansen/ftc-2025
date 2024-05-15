package org.firstinspires.ftc.teamcode.drive.advanced.subsystems;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

public class SwappableCameras {
    WebcamName webcam1, webcam2;
    Camera camera1, camera2;

    public VisionPortal visionPortal;
    public AprilTagProcessor aprilProcessor;
    public CSVisionProcessor visionProcessor;

    boolean selectedCameraTwo = false;

    public SwappableCameras(HardwareMap hardwareMap, Camera mainCamera, Camera secondaryCamera, Boolean auto) {
        int mult = auto ? 3 : 3;
        // Create the AprilTag processor.
        aprilProcessor = CameraStatic.baseAprilTags
                .setDrawAxes(Logging.DEBUG)
                .setDrawCubeProjection(Logging.DEBUG)
                .setDrawTagOutline(Logging.DEBUG)

//                These have been calculated from the data we collected
                .setLensIntrinsics(1485.792154 / mult, 1475.793189 / mult, 911.2968158 / mult, 513.6169402 / mult)
                .build();

        visionProcessor = new CSVisionProcessor();

        webcam1 = hardwareMap.get(WebcamName.class, mainCamera.getName());
        camera1 = mainCamera;
        webcam2 = hardwareMap.get(WebcamName.class, secondaryCamera.getName());
        camera2 = secondaryCamera;
        CameraName switchableCamera = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(webcam1, webcam2);

        // Create the vision portal by using a builder.
        visionPortal = CameraStatic.baseVisionPortal
                .setCamera(switchableCamera)
                .setCameraResolution(true ? new Size(640, 360) : new Size(1920, 1080)) // High quality for longer viewing

                .addProcessor(aprilProcessor)
                .addProcessor(visionProcessor)
                .build();

        visionPortal.setProcessorEnabled(visionProcessor, false);
    }

    public void selectCameraTwo(boolean selectedTwo) {
        if (selectedCameraTwo == selectedTwo) return;

        selectedCameraTwo = selectedTwo;
        if (selectedTwo) {
            visionPortal.setActiveCamera(webcam2);
            visionPortal.setProcessorEnabled(aprilProcessor, false);
            visionPortal.setProcessorEnabled(visionProcessor, true);
        } else {
            visionPortal.setActiveCamera(webcam1);
            visionPortal.setProcessorEnabled(aprilProcessor, true);
            visionPortal.setProcessorEnabled(visionProcessor, false);
        }
    }

    private boolean setExposure = true;

    public void update() {
        if (!setExposure && visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
            }

            exposureControl.setExposure(Camera.EXPOSURE, TimeUnit.MILLISECONDS);
            // Set Gain.
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            if (gainControl != null)
                gainControl.setGain(Camera.GAIN);

            setExposure = true;
        }

        (selectedCameraTwo ? camera2 : camera1).update(this);
    }
}
