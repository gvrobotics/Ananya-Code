package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class HoloOp_AN extends OpMode
{
    private double powerLX, powerLY, powerRX, powerRY;
    public double angle, magnitude;

    private double armPos = 0;
    private double addPos = 0.1;

    private DcMotor FL, BL, FR, BR;
    private Servo Arm;

    @Override
    public void init()
    {
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        Arm = hardwareMap.get(Servo.class, "Arm");

        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);


        BR.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        FL.setPower(0);
        Arm.setPosition(armPos);

    }

    @Override
    public void loop() {
        powerLX = gamepad1.left_stick_x;
        powerLY = gamepad1.left_stick_y;
        powerRX = gamepad1.right_stick_x;
        powerRY = gamepad1.right_stick_y;


        //Left-Stick Holonomic Drive

        //Calculate angle

        angle = Math.atan2(powerLY, powerLX);

        //Calculate magnitude

        magnitude = Math.sqrt( Math.pow(powerLX, 2) + Math.pow(powerLY, 2) )/2;

        //Drive

        FR.setPower(Math.sin(angle - 1/4*Math.PI) * magnitude);
        FL.setPower(Math.sin(angle + 1/4*Math.PI) * magnitude);
        BR.setPower(Math.sin(angle + 1/4*Math.PI) * magnitude);
        BL.setPower(Math.sin(angle - 1/4*Math.PI) * magnitude);

        //Right-Stick Turning

        //If left stick not moving
        if(magnitude < 0.1)
        {
            //If right stick is not up
            if(powerRY < 0.03 && powerRY > -0.03)
            {
                if (powerRX > 0.03 || powerRX < -0.03)
                {
                    FR.setPower(powerRX);
                    BR.setPower(powerRX);
                    BL.setPower(-powerRX);
                    FL.setPower(-powerRX);

                }
            }
        }

        if (gamepad2.dpad_up)
        {
            armPos += addPos;
        }

        if (gamepad2.dpad_down)
        {
            armPos -= addPos;
        }

        armPos = Range.clip (armPos, 0.0, 0.1);
        Arm.setPosition(armPos);


        telemetry.addData("RX", "%.2f", powerRX);
        telemetry.addData("RY", "%.2f", powerRY);
        telemetry.addData("LX", "%.2f", powerLX);
        telemetry.addData("LY", "%.2f", powerLY);
        telemetry.addData("Angle", "%.2f", angle);
        telemetry.addData("Magnitude", "%.2f", magnitude);
        telemetry.addData ("Arm Position","%.2f", armPos);
        telemetry.update();
    }
}