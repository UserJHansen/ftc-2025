package org.firstinspires.ftc.teamcode.drive.advanced.subsystems.cameras;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;

public interface Camera {
    long EXPOSURE = 5;
    int GAIN = 255;

    String getName();

    OpenGLMatrix getCameraPos();

    void update();
}
