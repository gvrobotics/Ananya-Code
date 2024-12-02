package org.firstinspires.ftc.teamcode.Auton;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous

public class AutonTemplate extends LinearOpMode {

    //CPI is calculated by (rotation / translational distance(circumference of wheel)) * (ticks / rotation)
    //To find Ticks: Clicks of Encoders per revolution
            // may change based off of different model types - you can get it from spec sheet: Encoder Resolution (in PPR)
    // Clicks of Encoders per revolution / (pi * diameter (in mm))

    double cpi = 5; // cycles per inch
    double cpd = 3.7; // clicks per degree

    public DcMotor BR, BL, FR, FL;

    public enum robotMotion
    {
        right, left;
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // hardware map
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");

        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);

        // set mode
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set initial power/position
        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);


        waitForStart();
        //TODO: Code Here!
        forward(0.5, 0.01);
        sleep(1000);
        movementRL(AutonTemplate.robotMotion.right, 90, 0.01);
        sleep(1000);
        movementRL(AutonTemplate.robotMotion.left, 90, 1);
        sleep(1000);
    }

    private void forward(double inch,  double power) {
        // calculate new position for (wheel) motors
        // make sure you cast to int!
        int a = (int)(FL.getCurrentPosition() + (inch * cpi));
        int b = (int)(FR.getCurrentPosition() + (inch * cpi));
        int c = (int)(BL.getCurrentPosition() + (inch * cpi));
        int d = (int)(BR.getCurrentPosition() + (inch * cpi));


        // sets new position for motors
        FL.setTargetPosition(a);
        FR.setTargetPosition(b);
        BL.setTargetPosition(c);
        BR.setTargetPosition(d);


        // sets desired power for motors
        // TODO: Do for each wheel motor
        FL.setPower(power);
        FR.setPower(power);
        BL.setPower(power);
        BR.setPower(power);


        // sets motors to RUN_TO_POSITION
        // TODO: Do for each wheel motor
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // loop to get telemetry while motors are running
        while (FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy()) {
            // TODO: Add telemetry data here
            telemetry.addData("FL Target", a);
            telemetry.addData("FL Actual", FL.getCurrentPosition());

            telemetry.addData("FR Target", b);
            telemetry.addData("FR Actual", FR.getCurrentPosition());

            telemetry.addData("BL Target", c);
            telemetry.addData("BL Actual", BL.getCurrentPosition());

            telemetry.addData("BR Target", d);
            telemetry.addData("BR Actual", BR.getCurrentPosition());

            telemetry.update();
        }

        // stop motors
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }

    private void backward(double inch,  double power) {
        // calculate new position for motors
        int a = (int)(FL.getCurrentPosition() - (inch * cpi));
        int b = (int)(FR.getCurrentPosition() - (inch * cpi));
        int c = (int)(BL.getCurrentPosition() - (inch * cpi));
        int d = (int)(BR.getCurrentPosition() - (inch * cpi));


        // sets new position for motors
        FL.setTargetPosition(-a);
        FR.setTargetPosition(-b);
        BL.setTargetPosition(-c);
        BR.setTargetPosition(-d);


        // sets desired power for motors
        FL.setPower(power);
        FR.setPower(power);
        BL.setPower(power);
        BR.setPower(power);


        // make motors run to position
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // loop to get telemetry while motors are running
        while (FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy()) {
            telemetry.addData("FL Target", a);
            telemetry.addData("FL Actual", FL.getCurrentPosition());

            telemetry.addData("FR Target", b);
            telemetry.addData("FR Actual", FR.getCurrentPosition());

            telemetry.addData("BL Target", c);
            telemetry.addData("BL Actual", BL.getCurrentPosition());

            telemetry.addData("BR Target", d);
            telemetry.addData("BR Actual", BR.getCurrentPosition());

            telemetry.update();
        }

        // stop motors
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }

    private void movementRL(robotMotion action, double degree,  double power) {

        if(action == robotMotion.left) {
            // left is moving backwards, right is moving forwards
            FL.setTargetPosition((int) (FL.getCurrentPosition() - (degree * cpd)));
            FR.setTargetPosition((int) (FR.getCurrentPosition() + (degree * cpd)));
            BL.setTargetPosition((int) (BL.getCurrentPosition() - (degree * cpd)));
            BR.setTargetPosition((int) (BR.getCurrentPosition() + (degree * cpd)));


            // sets desired power for motors
            FL.setPower(-power);
            FR.setPower(power);
            BL.setPower(-power);
            BR.setPower(power);


            // make motors run to position
            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // loop to get telemetry while motors are running
            while (FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy()) {
                telemetry.addLine("Turn Left");
                telemetry.update();
            }

            // stop motors
            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);
        }

        if(action == robotMotion.right) {
            // calculates new position for motors
            FL.setTargetPosition((int) (FL.getCurrentPosition() + (degree * cpd)));
            FR.setTargetPosition((int) (FR.getCurrentPosition() - (degree * cpd)));
            BL.setTargetPosition((int) (BL.getCurrentPosition() + (degree * cpd)));
            BR.setTargetPosition((int) (BR.getCurrentPosition() - (degree * cpd)));


            // sets desired power for motors
            FL.setPower(power);
            FR.setPower(-power);
            BL.setPower(power);
            BR.setPower(-power);


            // make motors run to position
            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // loop to get telemetry while motors are running
            while (FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy()) {
                telemetry.addLine("Turn Right");
                telemetry.update();
            }

            // stop motors
            FL.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            BR.setPower(0);
        }
    }
}   // end class