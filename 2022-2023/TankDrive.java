package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class TankDrive extends OpMode
{
    private double powerLX, powerLY, powerRX, powerRY;
private DcMotor FL, BL, FR, BR;

    @Override
    public void init()
    {

    }

    @Override
    public void loop()
    {
        powerLX = gamepad1.left_stick_x;
        powerLY = gamepad1.left_stick_y;
        powerRX = gamepad1.right_stick_x;
        powerRY = gamepad1.right_stick_y;

        if(powerLY > 0.07 || powerLY < -0.07)
        {
            FL.setPower(powerLY);
            BL.setPower(powerLY);
        } else
        {
            FL.setPower(0);
            BL.setPower(0);
        }

        if(powerRY > 0.07 || powerRY < -0.07)
        {
            FR.setPower(powerRY);
            BR.setPower(powerRY);
        } else
        {
            FR.setPower(0);
            BR.setPower(0);
        }
    }
}
