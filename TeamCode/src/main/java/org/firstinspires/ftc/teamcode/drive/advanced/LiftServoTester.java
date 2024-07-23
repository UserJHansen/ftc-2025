package org.firstinspires.ftc.teamcode.drive.advanced;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.advanced.subsystems.Logging;
import org.firstinspires.ftc.teamcode.drive.galahlib.Button;

@TeleOp(group = "advanced", name = "LiftServo Test")
@Config
public class LiftServoTester extends LinearOpMode {
    //    L
    public static double elbowLmax = 0; // DONE
    public static double elbowLmin = 0.93; // DONE
    //    R
    public static double elbowRmax = 1; // DONE
    public static double elbowRmin = 0.07; // DONE
    //    L
    public static double wristLmax = 0;
    public static double wristLmin = 0.5;
    //    R
    public static double wristRmax = 1;
    public static double wristRmin = 0;
    public static double clawmax = 1.0; // DONE
    public static double clawmin = 0.0; // DONE


    @Override
    public void runOpMode() throws InterruptedException {
        Button out = new Button(true);
        Servo elbowL = hardwareMap.get(Servo.class, "elbowL");
        Servo elbowR = hardwareMap.get(Servo.class, "elbowR");
        Servo wristL = hardwareMap.get(Servo.class, "wristL");
        Servo wristR = hardwareMap.get(Servo.class, "wristR");
        Servo claw = hardwareMap.get(Servo.class, "claw");

        Logging.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        while (!isStarted()) {
            sleep(10);
        }

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            out.update(gamepad1.back);
            if (out.val) {
                if (gamepad1.a) elbowL.setPosition(elbowLmax);
                if (gamepad1.b) elbowR.setPosition(elbowRmax);
                if (gamepad1.x) wristL.setPosition(wristLmax);
                if (gamepad1.y) wristR.setPosition(wristRmax);
                if (gamepad1.dpad_down) claw.setPosition(clawmax);
            } else {
                if (gamepad1.a) elbowL.setPosition(elbowLmin);
                if (gamepad1.b) elbowR.setPosition(elbowRmin);
                if (gamepad1.x) wristL.setPosition(wristLmin);
                if (gamepad1.y) wristR.setPosition(wristRmin);
                if (gamepad1.dpad_down) claw.setPosition(clawmin);
            }
        }
    }
}