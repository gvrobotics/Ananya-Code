package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class LeftAuton extends LinearOpMode {
    public DcMotor BR, BL, FR, FL, linear;
    public Servo claw;
    public ColorSensor color;
    double wpower = 0.1;
    double lpower = 0.2;
    double linearpower = 0.7;

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
        linear.setDirection(DcMotorSimple.Direction.FORWARD);

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

        claw.setPosition(0.45);
        sleep(1000);
        linearup(12, 0.7);
        sleep(1000);
        forward(15, 0.1);
        sleep(2000);
        int x = 1;
        int y = 2;
        int z = 3;
        if (color.blue() - 100 < color.red() && color.blue() + 100 > color.red())
            //red (Position one)
            {
                telemetry.addData("RED", x);
                telemetry.update();
                    //go to high junction with loaded cone
                forward(26,wpower);
                sleep(400);
                right(45,lpower);
                sleep(400);
                forward(1.5,wpower);
                sleep(400);


                    //drop loaded cone onto high junction
                linearup(21.5,linearpower);
                sleep(400);
                forward(2,wpower);
                sleep(400);
                lineardown(3,linearpower);
                sleep(400);
                claw.setPosition(0);
                sleep(400);
                backward(4,wpower);
                sleep(400);
                lineardown(23,linearpower);
                sleep(400);
                claw.setPosition(0.45);
                sleep(400);


                    //go to cone stack 1 and pick up cone 1
                left(145,lpower);
                sleep(400);
                forward(10,wpower);
                sleep(400);
                claw.setPosition(0.45);
                sleep(400);


                    //go to high junction 1
                backward(2,wpower);
                sleep(400);
                right(180,lpower);
                sleep(400);
                forward(9,wpower);
                sleep(400);
                left(45,lpower);
                sleep(400);
                forward(4,wpower);



                    //drop cone onto high junction 1
                linearup(22.5,linearpower);
                sleep(400);
                forward(1.5,lpower);
                sleep(400);
                lineardown(3,linearpower);
                sleep(400);
                claw.setPosition(0);
                sleep(400);
                backward(1,wpower);
                sleep(400);
                lineardown(33,linearpower);
                sleep(400);
                claw.setPosition(0.45);
                sleep(400);


                    //parking
                right(180,lpower);
                sleep(400);
                forward(33.5,wpower);
                sleep(400);
                left(90,lpower);
                sleep(400);
                forward(24,wpower);
                sleep(1000);
                claw.setPosition(0);
                sleep(1000);


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
                //go to high junction with loaded cone
                forward(26,wpower);
                sleep(400);
                right(45,lpower);
                sleep(400);
                forward(1.5,wpower);
                sleep(400);


                //drop loaded cone onto high junction
                linearup(21.5,linearpower);
                sleep(400);
                forward(2,wpower);
                sleep(400);
                lineardown(3,linearpower);
                sleep(400);
                claw.setPosition(0);
                sleep(400);
                backward(4,wpower);
                sleep(400);
                lineardown(23,linearpower);
                sleep(400);
                claw.setPosition(0.45);
                sleep(400);


                //go to cone stack 1 and pick up cone 1
                left(135,lpower);
                sleep(400);
                forward(10,wpower);
                sleep(400);
                claw.setPosition(0.45);
                sleep(400);


                //go to high junction 1
                backward(2,wpower);
                sleep(400);
                right(180,lpower);
                sleep(400);
                forward(36,wpower);
                sleep(400);
                left(90,lpower);
                sleep(400);
                forward(52,wpower);
                sleep(400);
                left(90,lpower);
                sleep(400);
                forward(38,wpower);
                sleep(400);
                right(90,lpower);
                sleep(400);


                //drop cone onto high junction 1
                linearup(22.5,linearpower);
                sleep(400);
                forward(1.5,lpower);
                sleep(400);
                lineardown(3,linearpower);
                sleep(400);
                claw.setPosition(0);
                sleep(400);
                backward(1,wpower);
                sleep(400);
                lineardown(33,linearpower);
                sleep(400);
                claw.setPosition(0.45);
                sleep(400);


                //parking
                right(180,lpower);
                sleep(400);
                forward(33.5,wpower);
                sleep(400);
                left(90,lpower);
                sleep(400);
                forward(24,wpower);
                sleep(1000);
                claw.setPosition(0);
                sleep(1000);


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
                //go to high junction with loaded cone
                forward(26,wpower);
                sleep(400);
                right(45,lpower);
                sleep(400);
                forward(1.5,wpower);
                sleep(400);


                //drop loaded cone onto high junction
                linearup(21.5,linearpower);
                sleep(400);
                forward(2,wpower);
                sleep(400);
                lineardown(3,linearpower);
                sleep(400);
                claw.setPosition(0);
                sleep(400);
                backward(4,wpower);
                sleep(400);
                lineardown(23,linearpower);
                sleep(400);
                claw.setPosition(0.45);
                sleep(400);


                //go to cone stack 1 and pick up cone 1
                left(135,lpower);
                sleep(400);
                forward(10,wpower);
                sleep(400);
                claw.setPosition(0.45);
                sleep(400);


                //go to high junction 1
                backward(2,wpower);
                sleep(400);
                right(180,lpower);
                sleep(400);
                forward(36,wpower);
                sleep(400);
                left(90,lpower);
                sleep(400);
                forward(52,wpower);
                sleep(400);
                left(90,lpower);
                sleep(400);
                forward(38,wpower);
                sleep(400);
                right(90,lpower);
                sleep(400);


                //drop cone onto high junction 1
                linearup(22.5,linearpower);
                sleep(400);
                forward(1.5,lpower);
                sleep(400);
                lineardown(3,linearpower);
                sleep(400);
                claw.setPosition(0);
                sleep(400);
                backward(1,wpower);
                sleep(400);
                lineardown(33,linearpower);
                sleep(400);
                claw.setPosition(0.45);
                sleep(400);


                //parking
                right(180,lpower);
                sleep(400);
                forward(33.5,wpower);
                sleep(400);
                left(90,lpower);
                sleep(400);
                forward(24,wpower);
                sleep(1000);
                claw.setPosition(0);
                sleep(1000);


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
        } else if (inch >= 26)
        {
            linear.setPower(0.4);
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

