package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RightAuton extends LinearOpMode {
    public DcMotor BR, BL, FR, FL, linear;
    public Servo claw;
    public ColorSensor color;
    double lpower = 0.25;
    double wpower = 0.4;

    //clicks per degree
    double cpd = 3;

    //clicks per inch
    double cpi = 12;

    //clicks per inch for linear slides since different diameter than wheels and different motor
    double lcpi = 194;

    @Override
    public void runOpMode() throws InterruptedException {
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        linear = hardwareMap.get(DcMotor.class, "L");
        claw = hardwareMap.get(Servo.class, "c");
        color = hardwareMap.colorSensor.get("Color");

        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        linear.setDirection(DcMotorSimple.Direction.REVERSE);

        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //  BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //  FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //  FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        linear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //   linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        BR.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        FL.setPower(0);
        linear.setPower(0);
        claw.setPosition(0);

        waitForStart();

        linearup(12, 0.6);
        sleep(1000);
        forward(21, 0.1);
        sleep(2000);
        int x = 1;
        int y = 2;
        int z = 3;
        if (color.blue() - 100 < color.red() && color.blue() + 100 > color.red())
        //red (Position one)
        {
            telemetry.addData("RED", x);
            telemetry.update();
            //go to the medium junction and drop cone
            forward(27, wpower);
            left(90,lpower);
            forward(11.5, wpower);
            linearup(11.5, lpower);
            forward(0.1, wpower);
            lineardown(20.5, lpower);
            claw.setPosition(0.45);
            backward(0.5, wpower);
            //go to cone stack and pick up cone
            right(90,lpower);
            forward(10.5, wpower);
            right(90,lpower);
            forward(46, wpower);
            linearup(7.5, lpower);
            claw.setPosition(0);
            //go to the low junction
            linearup(3.5, lpower);
            right(90,lpower);
            forward(10.5, wpower);
            right(90,lpower);
            forward(23, wpower);
            //drop cone onto low junction
            lineardown(13.5, lpower);
            claw.setPosition(0.45);
            //go to the parking zone
            backward(2,wpower);
            left(90,lpower);
            forward(5,wpower);
            right(90,lpower);
            forward(39.5,wpower);
            left(90,lpower);
            forward(5.5,wpower);
            //stop
            BR.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            FL.setPower(0);
            linear.setPower(0);

        } else if (color.green() > color.blue())
        //green (Position two)
        {
            telemetry.addData("GREEN", y);
            telemetry.update();
            //go to the medium junction and drop cone
            forward(27, wpower);
            left(90,lpower);
            forward(11.5, wpower);
            linearup(11.5, lpower);
            forward(0.1, wpower);
            lineardown(20.5, lpower);
            claw.setPosition(0.45);
            backward(0.5, wpower);
            //go to cone stack and pick up cone
            right(90,lpower);
            forward(10.5, wpower);
            right(90,lpower);
            forward(46, wpower);
            linearup(7.5, lpower);
            claw.setPosition(0);
            //go to the low junction
            linearup(3.5, lpower);
            right(90,lpower);
            forward(10.5, wpower);
            right(90,lpower);
            forward(23, wpower);
            //drop cone onto low junction
            lineardown(13.5, lpower);
            claw.setPosition(0.45);
            //go to the parking zone
            backward(2,wpower);
            left(90,lpower);
            forward(5,wpower);
            right(90,lpower);
            forward(14,wpower);
            right(90,lpower);
            forward(5.5,wpower);
            //stop
            BR.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            FL.setPower(0);
            linear.setPower(0);

        } else if (color.blue() > color.green())
        // blue (Position three)
        {
            telemetry.addData("BLUE", z);
            telemetry.update();
            //go to the medium junction and drop cone
            forward(27, wpower);
            left(90,lpower);
            forward(11.5, wpower);
            linearup(11.5, lpower);
            forward(0.1, wpower);
            lineardown(20.5, lpower);
            claw.setPosition(0.45);
            backward(0.5, wpower);
            //go to cone stack and pick up cone
            right(90,lpower);
            forward(10.5, wpower);
            right(90,lpower);
            forward(46, wpower);
            linearup(7.5, lpower);
            claw.setPosition(0);
            //go to the low junction
            linearup(3.5, lpower);
            right(90,lpower);
            forward(10.5, wpower);
            right(90,lpower);
            forward(23, wpower);
            //drop cone onto low junction
            lineardown(13.5, lpower);
            claw.setPosition(0.45);
            //go to the parking zone
            backward(10.5,wpower);
            right(90,lpower);
            //stop
            BR.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            FL.setPower(0);
            linear.setPower(0);


        } else //Fail
        {
            claw.setPosition(0.45);
        }

        BR.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        FL.setPower(0);
        linear.setPower(0);

    }


    private void forward(double inch, double power) {

        //Sets new position for motors
        int a = (int) (FL.getCurrentPosition() + (inch * cpi));
        int b = (int) (FR.getCurrentPosition() + (inch * cpi));
        int c = (int) (BL.getCurrentPosition() + (inch * cpi));
        int d = (int) (BR.getCurrentPosition() + (inch * cpi));

        FL.setTargetPosition(a);
        FR.setTargetPosition(b);
        BL.setTargetPosition(c);
        BR.setTargetPosition(d);

        //Sets desired power for motors
        FL.setPower(power);
        FR.setPower(power);
        BL.setPower(power);
        BR.setPower(power);

        //Makes the motors to run to the position
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Loop to run encoders method
        while (FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy()) {
            telemetry.addLine("Move Forward");
            telemetry.addData("Target", "%7d :%7d : %7d : %7d", a, b, c, d);
            telemetry.addData("Actual", "%7d :%7d : %7d : %7d", FL.getCurrentPosition(), FR.getCurrentPosition(), BL.getCurrentPosition(), BR.getCurrentPosition());
            telemetry.update();
        }

        //Eliminates momentum
        FL.setPower(-0.1);
        FR.setPower(-0.1);
        BL.setPower(-0.1);
        BR.setPower(-0.1);
        sleep(150);

        //Stop motors
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }

    private void backward(double inch, double power) {
        //Sets new position for motors
        int a = (int) (FL.getCurrentPosition() - (inch * cpi));
        int b = (int) (FR.getCurrentPosition() - (inch * cpi));
        int c = (int) (BL.getCurrentPosition() - (inch * cpi));
        int d = (int) (BR.getCurrentPosition() - (inch * cpi));

        FL.setTargetPosition(a);
        FR.setTargetPosition(b);
        BL.setTargetPosition(c);
        BR.setTargetPosition(d);

        //Sets desired power for motors
        FL.setPower(power);
        FR.setPower(power);
        BL.setPower(power);
        FL.setPower(power);

        //Makes the motors to run to the position
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Loop to run encoders method
        while (FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy()) {
            telemetry.addLine("Move Forward");
            telemetry.addData("Target", "%7d :%7d : %7d : %7d", a, b, c, d);
            telemetry.addData("Actual", "%7d :%7d : %7d : %7d", FL.getCurrentPosition(), FR.getCurrentPosition(), BL.getCurrentPosition(), BR.getCurrentPosition());
            telemetry.update();
        }

        //Eliminates momentum
        FL.setPower(0.1);
        FR.setPower(0.1);
        BL.setPower(0.1);
        BR.setPower(0.1);
        sleep(150);

        //Stop motors
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }

    private void right(double degree, double power) {
        //Sets new position for motors
        FL.setTargetPosition((int) (FL.getCurrentPosition() + (degree * cpd)));
        FR.setTargetPosition((int) (FR.getCurrentPosition() - (degree * cpd)));
        BL.setTargetPosition((int) (BL.getCurrentPosition() + (degree * cpd)));
        BR.setTargetPosition((int) (BR.getCurrentPosition() - (degree * cpd)));

        //Sets desired power for motors
        FL.setPower(power);
        FR.setPower(-power);
        BL.setPower(power);
        BR.setPower(-power);

        //Makes the motors to run to the position
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Loop to run encoders method
        while (FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy()) {

        }

        //Eliminates momentum
        FL.setPower(-0.1);
        FR.setPower(0.1);
        BL.setPower(-0.1);
        BR.setPower(0.1);
        sleep(150);

        //Stop motors
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }

    private void left(double degree, double power) {
        //Sets new position for motors
        FL.setTargetPosition((int) (FL.getCurrentPosition() - (degree * cpd)));
        FR.setTargetPosition((int) (FR.getCurrentPosition() + (degree * cpd)));
        BL.setTargetPosition((int) (BL.getCurrentPosition() - (degree * cpd)));
        BR.setTargetPosition((int) (BR.getCurrentPosition() + (degree * cpd)));

        //Sets desired power for motors
        FL.setPower(power);
        FR.setPower(-power);
        BL.setPower(power);
        FL.setPower(-power);

        //Makes the motors to run to the position
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Loop to run encoders method
        while (FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy()) {

        }

        //Eliminates momentum
        FL.setPower(-0.1);
        FR.setPower(0.1);
        BL.setPower(-0.1);
        BR.setPower(0.1);
        sleep(150);

        //Stop motors
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }


    private void linearup(double inch, double power) {
        int a = (int) (linear.getCurrentPosition() + (inch * lcpi));
        linear.setTargetPosition(a);
        linear.setPower(power);
        linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (linear.isBusy()) {
            telemetry.addLine("Linear up");
            telemetry.addData("Target", "%7d", a);
            telemetry.addData("Actual", "%7d", linear.getCurrentPosition());
            telemetry.update();
        }
        //Changes power based on height to stop gravity from pulling down linear slides
        if (inch < 8) {
            linear.setPower(0.1);
        } else if (inch >= 8 && inch < 18) {
            linear.setPower(0.2);
        } else if (inch >= 18) {
            linear.setPower(0.3);
        }
    }

    private void lineardown(double inch, double power) {
        int a = (int) (linear.getCurrentPosition() - (inch * lcpi));
        linear.setTargetPosition(a);
        linear.setPower(power);
        linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (linear.isBusy()) {
            telemetry.addLine("Linear up");
            telemetry.addData("Target", "%7d", a);
            telemetry.addData("Actual", "%7d", linear.getCurrentPosition());
            telemetry.update();
        }
        linear.setPower(0);
    }

    private void robotsleep(int sleep, double power) {
        BR.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        FL.setPower(0);
        linear.setPower(0);
    }

}
