package org.firstinspires.ftc.teamcode.drive.advanced;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.advanced.subsystems.Logging;

@TeleOp(group = "advanced", name = "Lift Position Test")
@Config
public class LiftPositionTester extends LinearOpMode {
    //    L
    public static double lLiftPosition = 0;
    public static double rLiftPosition = 0.93;


    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx lLift = hardwareMap.get(DcMotorEx.class, "liftL");
        DcMotorEx rLift = hardwareMap.get(DcMotorEx.class, "liftR");
        lLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lLift.setTargetPosition(0);
        lLift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        rLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rLift.setTargetPosition(0);
        rLift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rLift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        lLift.setDirection(DcMotorSimple.Direction.FORWARD);
        rLift.setDirection(DcMotorSimple.Direction.FORWARD);

        Logging.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        while (!isStarted()) {
            sleep(10);
        }

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            if (rLift.getCurrentPosition() < 1600 && lLift.getCurrentPosition() < 1600) {
                lLift.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
                rLift.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
            }

            Logging.LOG("lLift State: ", lLift.getCurrentPosition());
            Logging.LOG("rLift State: ", rLift.getCurrentPosition());
            Logging.update();
        }
    }
}