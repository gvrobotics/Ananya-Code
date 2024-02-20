package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class Holonomic_FINAL extends OpMode
{
    public DcMotor FR, FL, BR, BL;
    private double powerRY, powerRX, powerLX, powerLY, robotAngle, PowerMultiplier, lf, rb, rf, lb;

    @Override
    public void init()
    {
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");

        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);

        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
    }

    @Override
    public void loop()
    {
        powerLX = gamepad1.left_stick_x/2;
        powerLY = gamepad1.left_stick_y/2;
        powerRX = gamepad1.right_stick_x/2;
        powerRY = gamepad1.right_stick_y/2;

        if (gamepad1.right_stick_x != 0)
        {
            lf = powerRX;
            rb = -powerRX;
            lb = powerRX;
            rf = -powerRX;

            FR.setPower(rf);
            FL.setPower(lf);
            BR.setPower(rb);
            BL.setPower(lb);
        }
        else
        {
            robotAngle = Math.atan2(powerLX, powerLY);
            telemetry.addData("Robot angle:", robotAngle);
            telemetry.addData("powerRX: ", gamepad1.right_stick_x);
            telemetry.addData("powerRY: ", gamepad1.right_stick_y);
            telemetry.addData("powerLX: ", gamepad1.left_stick_x);
            telemetry.addData("powerLY: ", gamepad1.left_stick_y);

            telemetry.addData("FR: ", FR.getPower());
            telemetry.addData("FL: ", FL.getPower());
            telemetry.addData("BR: ", BR.getPower());
            telemetry.addData("BL: ", BL.getPower());
            telemetry.update();


            PowerMultiplier = Math.sqrt((Math.pow(powerLX, 2) + Math.pow(powerLY, 2)));

            lf = (PowerMultiplier*-1*(Math.sin(robotAngle-(Math.PI/4))));
            rb = (PowerMultiplier*-1*(Math.sin(robotAngle-(Math.PI/4))));
            lb = (PowerMultiplier*Math.sin(robotAngle+(Math.PI/4)));
            rf = (PowerMultiplier*Math.sin(robotAngle+(Math.PI/4)));

            FR.setPower(rf);
            FL.setPower(lf);
            BR.setPower(rb);
            BL.setPower(lb);
        }
    }
}
