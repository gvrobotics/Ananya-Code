package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class EncoderAuton_AN extends LinearOpMode {
    public DcMotor BR, BL, FR, FL;
    public Servo arm;

    //clicks per inch
    //CPI is calculated by ((Clicks of Encoders per revolution)/(diameter*pi   make sure it is in inches))
    //Clicks of Encoders per revolution may change based off of different model types
    //The smaller the diameter of the more clicks needed, the larger the diameter less clicks are needed
    double cpi = 5;

    //clicks per inch for linear slides since different diameter than wheels and different motor
    double lcpi = 68;

    //clicks per degree
    double cpd = 4;

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialization of motors and servos

        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        arm = hardwareMap.get(Servo.class, "Arm");

        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setDirection(Servo.Direction.REVERSE);

        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BR.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        FL.setPower(0);
        arm.setPosition(0);

      waitForStart();
      sleep(200);
      forward (2, 0.03);
      sleep(200);
/*
        //Forward 12 inches
        forward(12, 0.4);

        //Backward 12 inches
        backward(12,0.4);

        //Right 90 degrees
        right(90,0.4);

        //Left 90 degrees
        left(90,0.4);

        //Linear slides up 12 inches
        linearup(12,0.6);

        //Linear slides down 12 inches
        lineardown(12,0.6);

 */

        //Stops the robot for the rest of auton
        BR.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        FL.setPower(0);
        arm.setPosition(0);

    }


    private void forward(double inch, double power) {
        //Sets new position for motors
        int a = (int) (FL.getCurrentPosition() + (inch * cpi));
        int b = (int) (FR.getCurrentPosition() + (inch * cpi));
        int c = (int) (BL.getCurrentPosition() + (inch * cpi));
        int d = (int) (BR.getCurrentPosition() + (inch * cpi));

        //Creates a target position
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
            //Displays on the phone what the target position is vs the actual position
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

        FL.setTargetPosition(-a);
        FR.setTargetPosition(-b);
        BL.setTargetPosition(-c);
        BR.setTargetPosition(-d);

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
        FL.setTargetPosition((int) (FL.getCurrentPosition() - (degree * cpd)));
        FR.setTargetPosition((int) (FR.getCurrentPosition() + (degree * cpd)));
        BL.setTargetPosition((int) (BL.getCurrentPosition() - (degree * cpd)));
        BR.setTargetPosition((int) (BR.getCurrentPosition() + (degree * cpd)));

        //Sets desired power for motors
        FL.setPower(-power);
        FR.setPower(power);
        BL.setPower(-power);
        FL.setPower(power);

        //Makes the motors to run to the position
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Loop to run encoders method
        while (FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy()) {

        }

        //Eliminates momentum
        FL.setPower(0.1);
        FR.setPower(-0.1);
        BL.setPower(0.1);
        BR.setPower(-0.1);
        sleep(150);

        //Stop motors
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }

    private void left(double degree, double power) {
        //Sets new position for motors
        FL.setTargetPosition((int) (FL.getCurrentPosition() + (degree * cpd)));
        FR.setTargetPosition((int) (FR.getCurrentPosition() - (degree * cpd)));
        BL.setTargetPosition((int) (BL.getCurrentPosition() + (degree * cpd)));
        BR.setTargetPosition((int) (BR.getCurrentPosition() - (degree * cpd)));

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
}

    /*
    private void linearup(double inch, double power)
    {
        int a = (int) (arm.getCurrentPosition() + (inch*lcpi));
        arm.setTargetPosition(a);
        arm.setPower(power);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (arm.isBusy())
        {
            telemetry.addLine("Linear up");
            telemetry.addData("Target", "%7d", a);
            telemetry.addData("Actual", "%7d", arm.getCurrentPosition());
            telemetry.update();
        }

        //Changes power based on height to stop gravity from pulling down linear slides
        if(inch < 8)
        {
            arm.setPower(0.1);
        }else if (inch >= 8 && inch < 18)
        {
            arm.setPower(0.2);
        }else if(inch >= 18)
        {
            arm.setPower(0.3);
        }
    }
    private void lineardown(double inch, double power)
    {
        int a = (int) (arm.getCurrentPosition() - (inch*lcpi));
        arm.setTargetPosition(a);
        arm.setPower(power);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (arm.isBusy())
        {
            telemetry.addLine("Linear up");
            telemetry.addData("Target", "%7d", a);
            telemetry.addData("Actual", "%7d", arm.getCurrentPosition());
            telemetry.update();
        }
        arm.setPower(0);
    }
}
     */