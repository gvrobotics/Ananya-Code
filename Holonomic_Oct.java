package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class Holonomic_Oct extends OpMode
{
    public DcMotor FR, FL, BR, BL;
    private double powerRY, powerRX, powerLX, powerLY, robotAngle, PowerMultiplier, fl, br, fr, bl;

    @Override
    public void init()
    {
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");

        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
    }

    @Override
    public void loop()
    {
        powerLX = gamepad1.left_stick_x;
        powerLY = gamepad1.left_stick_y;
        powerRX = gamepad1.right_stick_x;
        powerRY = gamepad1.right_stick_y;

        if (gamepad1.right_bumper || gamepad1.left_bumper)
        {
            powerLX = gamepad1.left_stick_x/2;
            powerLY = gamepad1.left_stick_y/2;
            powerRX = gamepad1.right_stick_x/2;
            powerRY = gamepad1.right_stick_y/2;
        }

        if (gamepad1.left_stick_x != 0)
        {
            fl = powerRY;
            br = -powerRY;
            bl = powerRY;
            fr = -powerRY;

            FR.setPower(fr);
            FL.setPower(fl);
            BR.setPower(br);
            BL.setPower(bl);
        }
        else
        {
            robotAngle = Math.atan2(powerRX, powerRY);
            telemetry.addData("Robot angle:", robotAngle);
            telemetry.update();

            PowerMultiplier = Math.sqrt((Math.pow(powerRX, 2) + Math.pow(powerRY, 2)));

            if(powerRX == 0 || powerRY == 0)
            {
                if (powerRY <= 1 && powerRX == 0)
                {
                    fl = powerRY;
                    br = powerRY;
                    bl = powerRY;
                    fr = powerRY;

                    FR.setPower(fr);
                    FL.setPower(fl);
                    BR.setPower(br);
                    BL.setPower(bl);
                }
                else if (powerRX <= 1 && powerRY == 0)
                {
                    fl = powerRX;
                    br = powerRX;
                    bl = -powerRX;
                    fr = -powerRX;

                    FR.setPower(fr);
                    FL.setPower(fl);
                    BR.setPower(br);
                    BL.setPower(bl);
                }
            }
            fl = (PowerMultiplier*(Math.sin(robotAngle+(Math.PI/4))));
            br = (PowerMultiplier*(Math.sin(robotAngle+(Math.PI/4))));
            bl = (PowerMultiplier*-1*Math.sin(robotAngle-(Math.PI/4)));
            fr = (PowerMultiplier*-1*Math.sin(robotAngle-(Math.PI/4)));

            FR.setPower(fr);
            FL.setPower(fl);
            BR.setPower(br);
            BL.setPower(bl);
        }
    }
}