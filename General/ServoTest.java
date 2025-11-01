package org.firstinspires.ftc.teamcode.Tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name = "Servo Test Forward", group = "Servo")
public class ServoTest extends OpMode {
    private Servo servo;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "f");

        servo.setDirection(Servo.Direction.FORWARD);

        servo.setPosition(0);
    }

    @Override
    public void loop() {
        if (gamepad1.left_bumper) {
            servo.setPosition(0);
        }
        if (gamepad1.right_bumper) {
            servo.setPosition(1);
        }
        if (gamepad1.dpad_up) {
            servo.setPosition(0.2);
        }
        if (gamepad1.dpad_left) {
            servo.setPosition(0.3);
        }
        if (gamepad1.dpad_down) {
            servo.setPosition(0.4);
        }
        if (gamepad1.dpad_right) {
            servo.setPosition(0.5);
        }
        if (gamepad1.y) {
            servo.setPosition(0.6);
        }
        if (gamepad1.x) {
            servo.setPosition(0.7);
        }
        if (gamepad1.a) {
            servo.setPosition(0.8);
        }
        if (gamepad1.b) {
            servo.setPosition(0.9);
        }

        telemetry.addData("Position: ", servo.getPosition());
        telemetry.update();
    }
}
