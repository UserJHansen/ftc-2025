package org.firstinspires.ftc.teamcode.drive.advanced.subsystems;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;


@Config
public class CSVisionProcessor implements VisionProcessor {
    public static int width = 40;
    public static int height = 40;
    public static int leftX = 180;
    public static int leftY = 220;
    public static int middleX = 350;
    public static int middleY = 200;
    public static int rightX = 515;
    public static int rightY = 200;
    static StartingPosition selection = StartingPosition.NONE;
    static Mat submat = new Mat();
    static Mat hsvMat = new Mat();

    public CSVisionProcessor() {

    }

    public static Rect getLeft() {
        return new Rect(leftX, leftY, width, height);
    }

    public static Rect getMiddle() {
        return new Rect(middleX, middleY, width, height);
    }

    public static Rect getRight() {
        return new Rect(rightX, rightY, width, height);
    }

    public static int getIntPosition() {
        StartingPosition pos = CSVisionProcessor.getStartingPosition();

        if (pos == StartingPosition.LEFT) {
            return 1;
        } else if (pos == StartingPosition.CENTER) {
            return 2;
        } else if (pos == StartingPosition.RIGHT) {
            return 3;
        }

        return 0;
    }

    public static StartingPosition getStartingPosition() {
        return selection;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);

        double satRectLeft = getAvgSaturation(hsvMat, getLeft());
        double satRectMiddle = getAvgSaturation(hsvMat, getMiddle());
        double satRectRight = getAvgSaturation(hsvMat, getRight());

        if ((satRectLeft > satRectMiddle) && (satRectLeft > satRectRight)) {
            selection = StartingPosition.LEFT;

        } else if ((satRectMiddle > satRectLeft) && (satRectMiddle > satRectRight)) {
            selection = StartingPosition.CENTER;

        } else if ((satRectRight > satRectMiddle) && (satRectRight > satRectLeft)) {
            selection = StartingPosition.RIGHT;
        } else {

            selection = StartingPosition.NONE;
        }

        return selection;
    }

    protected double getAvgSaturation(Mat input, Rect rect) {
        submat = input.submat(rect);
        Scalar color = Core.mean(submat);
        return color.val[1];
    }

    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);

        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);

        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint selectedPaint = new Paint();
        selectedPaint.setColor(Color.RED);
        selectedPaint.setStyle(Paint.Style.STROKE);
        selectedPaint.setStrokeWidth(scaleCanvasDensity * 4);

        Paint nonSelected = new Paint();
        nonSelected.setStrokeWidth(scaleCanvasDensity * 4);
        nonSelected.setStyle(Paint.Style.STROKE);
        nonSelected.setColor(Color.GREEN);

        android.graphics.Rect drawRectangleLeft = makeGraphicsRect(getLeft(), scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectangleMiddle = makeGraphicsRect(getMiddle(), scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectangleRight = makeGraphicsRect(getRight(), scaleBmpPxToCanvasPx);

        selection = (StartingPosition) userContext;

        canvas.drawRect(drawRectangleLeft, selection == StartingPosition.LEFT ? selectedPaint : nonSelected);
        canvas.drawRect(drawRectangleMiddle, selection == StartingPosition.CENTER ? selectedPaint : nonSelected);
        canvas.drawRect(drawRectangleRight, selection == StartingPosition.RIGHT ? selectedPaint : nonSelected);
    }

    public enum StartingPosition {
        NONE,
        LEFT,
        RIGHT,
        CENTER
    }

}