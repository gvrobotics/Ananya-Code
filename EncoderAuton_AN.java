package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class EncoderAuton_AN extends LinearOpMode
{
    public DcMotor BR, BL, FR, FL, Arm0, Arm1;

    //clicks per inch
    //CPI is calculated by ((Clicks of Encoders per revolution)/(diameter*pi   make sure it is in inches))
    //Clicks of Encoders per revolution may change based off of different model types
    //The smaller the diameter of the more clicks needed, the larger the diameter less clicks are needed
    //goBilda motor encoder resolution or CPR : 145.6 OR 233
    //diamater of wheels : 96 mm / 3.77953 inches
    //145.6 / (3.77953 * pi) = 12.262
    double cpi = 7.7; //18.78

    //clicks per inch for linear slides since different diameter than wheels and different motor
    //double lcpi = 68;

    //clicks per degree
    double cpd = 4; //3.19444444444

    @Override
    public void runOpMode() throws InterruptedException
    {
        //Initialization of motors and servos

        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        Arm0 = hardwareMap.get(DcMotor.class, "Arm0");
        Arm1 = hardwareMap.get(DcMotor.class, "Arm1");


        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        Arm0.setDirection(DcMotorSimple.Direction.REVERSE);
        Arm1.setDirection(DcMotorSimple.Direction.FORWARD);


        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /*
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         */

        BR.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        FL.setPower(0);
        Arm0.setPower(0);
        Arm1.setPower(0);

        waitForStart();

        //Closing claw
        //claw.setPosition(0.45);
        forward(2, 0.2);
        //Opening claw
        //claw.setPosition(0);

        //Forward 12 inches
        /*
        forward(44, 0.4);

        //Backward 12 inches
        backward(8,0.4);

        //Right 90 degrees
        right(90,0.4);

        //Left 90 degrees
        left(90,0.4);

        //Linear slides up 12 inches
        //linearup(12,0.6);

        //Linear slides down 12 inches
        //lineardown(12,0.6);
*/
        //Stops the robot for the rest of auton
        BR.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        FL.setPower(0);
        Arm0.setPower(0);
        Arm1.setPower(0);
    }

    private void forward(double inch,  double power)
    {
        //Sets new position for motors
        int a = (int) (FL.getCurrentPosition() + (inch*cpi));
        int b = (int) (FR.getCurrentPosition() + (inch*cpi));
        int c = (int) (BL.getCurrentPosition() + (inch*cpi));
        int d = (int) (BR.getCurrentPosition() + (inch*cpi));

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
        while (FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy())
        {
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

    private void backward(double inch, double power)
    {
        //Sets new position for motors
        int a = (int) (FL.getCurrentPosition() - (inch*cpi));
        int b = (int) (FR.getCurrentPosition() - (inch*cpi));
        int c = (int) (BL.getCurrentPosition() - (inch*cpi));
        int d = (int) (BR.getCurrentPosition() - (inch*cpi));

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
        while (FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy())
        {
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


    /*
    private void armup(double degree, double power)
    {
        int a = (int) (Arm0.getCurrentPosition() + (inch*lcpi));
        int b = (int) (Arm1.getCurrentPosition() + (inch*lcpi));
        Arm0.setTargetPosition(a);
        Arm1.setTargetPosition(b);
        Arm0.setPower(power);
        Arm1.setPower(power);
        Arm0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (Arm0.isBusy() && Arm1.isBusy()) {
            telemetry.addLine("Arm up");
            telemetry.addData("Target", "%7d :%7d, a, b);
            telemetry.addData("Actual", "%7d": %7d, Arm0.getCurrentPosition(), Arm1.getCurrentPosition());
            telemetry.update();
        }
    }

        //PROBABLY DONT NEED THIS
        /*
        //Changes power based on height to stop gravity from pulling down linear slides
        if(inch < 8)
        {
            linear.setPower(0.1);
        }else if (inch >= 8 && inch < 18)
        {
            linear.setPower(0.2);
        }else if(inch >= 18)
        {
            linear.setPower(0.3);
        }
        */

    /*
    }
    private void armdown(double inch, double power)
    {
       int a = (int) (Arm0.getCurrentPosition() - (inch*lcpi));
        int b = (int) (Arm1.getCurrentPosition() - (inch*lcpi));
        Arm0.setTargetPosition(a);
        Arm1.setTargetPosition(b);
        Arm0.setPower(power);
        Arm1.setPower(power);
        Arm0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (Arm0.isBusy() && Arm1.isBusy()) {
            telemetry.addLine("Arm down");
            telemetry.addData("Target", "%7d :%7d, a, b);
            telemetry.addData("Actual", "%7d": %7d, Arm0.getCurrentPosition(), Arm1.getCurrentPosition());
            telemetry.update();
         }
        Arm0.setPower(0);
        Arm1.setPower(0);
    }
     */
}