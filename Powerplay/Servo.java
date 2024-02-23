package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class Servo extends OpMode {
    public com.qualcomm.robotcore.hardware.Servo claw;
    public DcMotor linear;

    @Override
    public void init() {
        linear = hardwareMap.get(DcMotor.class, "L");
        claw = hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "c");

        linear.setDirection(DcMotorSimple.Direction.FORWARD);

        linear.setPower(0);
        claw.setPosition(0);
    }

    @Override
    public void loop() {

    }

    {
        if (gamepad1.dpad_up) {
            linear.setPower(0.7);
        } else if (gamepad1.dpad_up) {
            linear.setPower(-0.7);

        }

        if (gamepad1.a) {
            claw.setPosition(0.5);
        } else if (gamepad1.y) {
            claw.setPosition(0);

        }

    }
}
