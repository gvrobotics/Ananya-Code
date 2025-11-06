package org.firstinspires.ftc.teamcode.Tele;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp (name = "Simple Mecanum Drive")
public class SimpleMecanumTemplate extends OpMode
{
    public DcMotor BR, BL, FR, FL;
    private double powerRX, powerLX, powerLY;
   
    @Override
    public void init() {
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");

        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);

        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BR.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        FL.setPower(0);
    }

    @Override
    public void loop() {
        // Read joystick inputs
        powerLX = gamepad1.left_stick_x;
        powerLY = gamepad1.left_stick_y;
        powerRX = gamepad1.right_stick_x;

        // mecanum drive components
        double lf = powerLY + powerLX + powerRX;
        double rf = powerLY - powerLX - powerRX;
        double lb = powerLY - powerLX + powerRX;
        double rb = powerLY + powerLX - powerRX;

        // Apply powers
        FL.setPower(lf);
        FR.setPower(rf);
        BL.setPower(lb);
        BR.setPower(rb);
        

        // Telemetry for debugging
        telemetry.addData("LX", powerLX);
        telemetry.addData("LY", powerLY);
        telemetry.addData("RX", powerRX);
        telemetry.addData("FL", FL.getPower());
        telemetry.addData("FR", FR.getPower());
        telemetry.addData("BL", BL.getPower());
        telemetry.addData("BR", BR.getPower());
        telemetry.update();
    }
}
