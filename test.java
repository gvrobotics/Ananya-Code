package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class test extends OpMode
{
    public DcMotor one, two;
    private Servo move;

    @Override
    public void init()
    {
        one = hardwareMap.get(DcMotor.class, "1");
        two = hardwareMap.get(DcMotor.class, "2");
        move = hardwareMap.get(Servo.class, "sp");

        one.setDirection(DcMotorSimple.Direction.REVERSE);
        two.setDirection(DcMotorSimple.Direction.FORWARD);
        move.setDirection(Servo.Direction.FORWARD);

        one.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        two.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        one.setPower(0);
        two.setPower(0);
        move.setPosition(0.7);
    }

    @Override
    public void loop()
    {

        telemetry.addData("one: ", one.getPower());
        telemetry.addData("two: ", two.getPower());
        telemetry.addData("move: ", move.getPosition());
        telemetry.update();

        if (gamepad1.dpad_up) {
            move.setPosition(0.7);
        } else if (gamepad1.dpad_down) {
            move.setPosition(0.5);
        } else if (gamepad1.dpad_left) {
            move.setPosition(0.3);
        }

        if (gamepad2.dpad_up) {
            one.setPower(0.5);
            two.setPower(0.5);
        } else if (gamepad2.dpad_down) {
            one.setPower(0.4);
            two.setPower(0.4);
        } else if (gamepad2.dpad_left) {
            one.setPower(0.3);
            two.setPower(0.3);
        } else if (gamepad2.dpad_right) {
            one.setPower(0.2);
            two.setPower(0.2);
        } else if (gamepad2.a) {
            one.setPower(0);
            two.setPower(0);
        }
    }
}