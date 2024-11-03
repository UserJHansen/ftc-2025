package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.galahlib.Button;
import org.firstinspires.ftc.teamcode.staticData.Logging;

@TeleOp(group = "testing", name = "Motor Identifier Test")
@Disabled
public class MotorIdentifier extends LinearOpMode {
    public int hubNumber = 0;
    public int motorNumber = 0;
    public TestDevice deviceToTest = TestDevice.Motor;

    @Override
    public void runOpMode() throws InterruptedException {
        Button previousHub = new Button(true, (none) -> this.hubNumber = this.hubNumber == 0 ? 1 : 0);
        Button nextHub = new Button(true, (none) -> this.hubNumber = this.hubNumber == 0 ? 1 : 0);
        Button previousDevice = new Button(true, (none) -> this.deviceToTest = this.deviceToTest.previous);
        Button nextDevice = new Button(true, (none) -> this.deviceToTest = this.deviceToTest.next);
        Button previousMotor = new Button(true, (none) -> {
            this.motorNumber -= 1;
            if (this.motorNumber < 0) this.motorNumber = this.deviceToTest.maxId;
        });
        Button nextMotor = new Button(true, (none) -> {
            this.motorNumber += 1;
            if (this.motorNumber > this.deviceToTest.maxId) this.motorNumber = 0;
        });

        Logging.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        while (!isStarted()) {
            sleep(10);
        }

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            previousHub.update(gamepad1.dpad_down);
            nextHub.update(gamepad1.dpad_up);
            previousDevice.update(gamepad1.dpad_left);
            nextDevice.update(gamepad1.dpad_right);
            previousMotor.update(gamepad1.left_bumper);
            nextMotor.update(gamepad1.right_bumper);

            double power = gamepad1.right_trigger - gamepad1.left_trigger;
            switch (this.deviceToTest) {
                case Motor:
                    DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "H" + hubNumber + "M" + motorNumber);
                    motor.setPower(power);
                    Logging.LOG("Motor Current Draw", motor.getCurrent(CurrentUnit.AMPS));
                    Logging.LOG("Motor Position", motor.getCurrentPosition());
                    break;
                case Servo:
                    Servo servo = hardwareMap.get(Servo.class, "H" + hubNumber + "S" + motorNumber);
                    servo.setPosition(power);
                    break;
            }

            Logging.LOG("Currently selected Power", power);

            Logging.LOG("");

            Logging.LOG("Currently selected Hub", hubNumber);
            Logging.LOG("Currently selected Device", deviceToTest.name());
            Logging.LOG("Currently selected Number", motorNumber);

            Logging.update();
        }
    }

    public enum TestDevice {
        Motor,
        Servo;

        static {
            Motor.next = Servo;
            Motor.previous = Servo;
            Motor.maxId = 3;
            Servo.next = Motor;
            Servo.previous = Motor;
            Servo.maxId = 5;
        }

        public TestDevice next;
        public TestDevice previous;
        public int maxId;
    }
}