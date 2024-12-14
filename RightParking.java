package org.firstinspires.ftc.teamcode.Auton;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous

public class RightParking extends LinearOpMode {

    double cpi = 60; // cycles per inch
    double cpd = 11; // clicks per degree

    public DcMotor BR, BL, FR, FL;
    public Servo Claw;

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
        Claw = hardwareMap.get(Servo.class, "Claw");

        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        Claw.setDirection(Servo.Direction.REVERSE);

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
        BR.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        FL.setPower(0);
        Claw.setPosition(0.28);


        //motor powers:
            //forward/backward: 0.5
            //strafe: 0.5
            //right/left: 0.3
            //claw open: 0.15

        waitForStart();
        forward(5, 0.5);
        sleep(500);
        strafeRight(30, 0.5);
        sleep(500);
        backward(4.5, 0.5);
        sleep(500);
    }

    private void forward(double inch,  double power) {
        // calculate new position for (wheel) motors
        int a = (int)(BR.getCurrentPosition() + (inch * cpi));
        int b = (int)(BL.getCurrentPosition() + (inch * cpi));
        int c = (int)(FR.getCurrentPosition() + (inch * cpi));
        int d = (int)(FL.getCurrentPosition() + (inch * cpi));

        // sets new position for motors
        BR.setTargetPosition(a);
        BL.setTargetPosition(b);
        FL.setTargetPosition(c);
        FR.setTargetPosition(d);

        // sets desired power for motors
        BR.setPower(power);
        BL.setPower(power);
        FR.setPower(power);
        FL.setPower(power);

        // make motors run to position
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // loop to get telemetry while motors are running
        while (BR.isBusy() && BL.isBusy() && FR.isBusy() && FL.isBusy()) {
            telemetry.addLine("Forward");

            telemetry.addData("BR Target", a);
            telemetry.addData("BL Target", b);
            telemetry.addData("FR Target", c);
            telemetry.addData("FL Target", d);

            telemetry.addData("BR Current", BR.getCurrentPosition());
            telemetry.addData("BL Current", BL.getCurrentPosition());
            telemetry.addData("FR Current", FR.getCurrentPosition());
            telemetry.addData("FL Current", FL.getCurrentPosition());

            telemetry.update();
        }
        // stop motors
        BR.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        FL.setPower(0);
    }

    private void backward(double inch,  double power) {
        // calculate new position for (wheel) motors
        int a = (int)(BR.getCurrentPosition() - (inch * cpi));
        int b = (int)(BL.getCurrentPosition() - (inch * cpi));
        int c = (int)(FR.getCurrentPosition() - (inch * cpi));
        int d = (int)(FL.getCurrentPosition() - (inch * cpi));

        // sets new position for motors
        BR.setTargetPosition(a);
        BL.setTargetPosition(b);
        FR.setTargetPosition(c);
        FL.setTargetPosition(d);

        // sets desired power for motors
        BR.setPower(power);
        BL.setPower(power);
        FR.setPower(power);
        FL.setPower(power);

        // make motors run to position
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // loop to get telemetry while motors are running
        while (BR.isBusy() && BL.isBusy() && FR.isBusy() && FL.isBusy()) {
            telemetry.addLine("Backward");

            telemetry.addData("BR Target", a);
            telemetry.addData("BL Target", b);
            telemetry.addData("FR Target", c);
            telemetry.addData("FL Target", d);

            telemetry.addData("BR Current", BR.getCurrentPosition());
            telemetry.addData("BL Current", BL.getCurrentPosition());
            telemetry.addData("FR Current", FR.getCurrentPosition());
            telemetry.addData("FL Current", FL.getCurrentPosition());

            telemetry.update();
        }
        // stop motors
        BR.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        FL.setPower(0);
    }

    private void strafeLeft (double inch,  double power) {
        // FL and BR go backward, FR and BL go forward
        BR.setTargetPosition((int) (BR.getCurrentPosition() - (inch * cpi)));
        BL.setTargetPosition((int) (BL.getCurrentPosition() + (inch * cpi)));
        FR.setTargetPosition((int) (FR.getCurrentPosition() + (inch * cpi)));
        FL.setTargetPosition((int) (FL.getCurrentPosition() - (inch * cpi)));

        // sets desired power for motors
        BR.setPower(-power);
        BL.setPower(power);
        FR.setPower(power);
        FL.setPower(-power);

        // make motors run to position
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // loop to get telemetry while motors are running
        while (BR.isBusy() && BL.isBusy() && FR.isBusy() && FL.isBusy()) {
            telemetry.addLine("Strafe Left");

            telemetry.addData("BR Target", BR.getTargetPosition());
            telemetry.addData("BL Target", BL.getTargetPosition());
            telemetry.addData("FR Target", FR.getTargetPosition());
            telemetry.addData("FL Target", FL.getTargetPosition());

            telemetry.addData("BR Current", BR.getCurrentPosition());
            telemetry.addData("BL Current", BL.getCurrentPosition());
            telemetry.addData("FR Current", FR.getCurrentPosition());
            telemetry.addData("FL Current", FL.getCurrentPosition());

            telemetry.update();
        }
        // stop motors
        BR.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        FL.setPower(0);
    }

    private void strafeRight (double inch,  double power) {
        // FR and BL go backward, FL and BR go forward
        BR.setTargetPosition((int) (BR.getCurrentPosition() + (inch * cpi)));
        BL.setTargetPosition((int) (BL.getCurrentPosition() - (inch * cpi)));
        FR.setTargetPosition((int) (FR.getCurrentPosition() - (inch * cpi)));
        FL.setTargetPosition((int) (FL.getCurrentPosition() + (inch * cpi)));

        // sets desired power for motors
        BR.setPower(power);
        BL.setPower(-power);
        FR.setPower(-power);
        FL.setPower(power);

        // make motors run to position
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // loop to get telemetry while motors are running
        while (BR.isBusy() && BL.isBusy() && FR.isBusy() && FL.isBusy()) {
            telemetry.addLine("Strafe Right");

            telemetry.addData("BR Target", BR.getTargetPosition());
            telemetry.addData("BL Target", BL.getTargetPosition());
            telemetry.addData("FR Target", FR.getTargetPosition());
            telemetry.addData("FL Target", FL.getTargetPosition());

            telemetry.addData("BR Current", BR.getCurrentPosition());
            telemetry.addData("BL Current", BL.getCurrentPosition());
            telemetry.addData("FR Current", FR.getCurrentPosition());
            telemetry.addData("FL Current", FL.getCurrentPosition());

            telemetry.update();
        }
        // stop motors
        BR.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        FL.setPower(0);
    }

    private void movementRL(robotMotion action, double degree,  double power) {

        if(action == robotMotion.left) {
            // left is moving backwards, right is moving forwards
            BR.setTargetPosition((int) (BR.getCurrentPosition() + (degree * cpd)));
            BL.setTargetPosition((int) (BL.getCurrentPosition() - (degree * cpd)));
            FR.setTargetPosition((int) (FR.getCurrentPosition() + (degree * cpd)));
            FL.setTargetPosition((int) (FL.getCurrentPosition() - (degree * cpd)));

            // sets desired power for motors
            BR.setPower(power);
            BL.setPower(-power);
            FR.setPower(power);
            FL.setPower(-power);

            // make motors run to position
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // loop to get telemetry while motors are running
            while (BR.isBusy() && BL.isBusy() && FR.isBusy() && FL.isBusy()) {
                telemetry.addLine("Turning Left");

                telemetry.addData("BR Target", BR.getTargetPosition());
                telemetry.addData("BL Target", BL.getTargetPosition());
                telemetry.addData("FR Target", FR.getTargetPosition());
                telemetry.addData("FL Target", FL.getTargetPosition());

                telemetry.addData("BR Current", BR.getCurrentPosition());
                telemetry.addData("BL Current", BL.getCurrentPosition());
                telemetry.addData("FR Current", FR.getCurrentPosition());
                telemetry.addData("FL Current", FL.getCurrentPosition());

                telemetry.update();
            }
            // stop motors
            BR.setPower(0);
            BL.setPower(0);
            FR.setPower(0);
            FL.setPower(0);
        }

        if(action == robotMotion.right) {
            // right is moving backwards, left is moving forwards
            BR.setTargetPosition((int) (BR.getCurrentPosition() - (degree * cpd)));
            BL.setTargetPosition((int) (BL.getCurrentPosition() + (degree * cpd)));
            FR.setTargetPosition((int) (FR.getCurrentPosition() - (degree * cpd)));
            FL.setTargetPosition((int) (FL.getCurrentPosition() + (degree * cpd)));

            // sets desired power for motors
            BR.setPower(-power);
            BL.setPower(power);
            FR.setPower(-power);
            FL.setPower(power);

            // make motors run to position
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // loop to get telemetry while motors are running
            while (BR.isBusy() && BL.isBusy() && FR.isBusy() && FL.isBusy()) {
                telemetry.addLine("Turning Right");

                telemetry.addData("BR Target", BR.getTargetPosition());
                telemetry.addData("BL Target", BL.getTargetPosition());
                telemetry.addData("FR Target", FR.getTargetPosition());
                telemetry.addData("FL Target", FL.getTargetPosition());

                telemetry.addData("BR Current", BR.getCurrentPosition());
                telemetry.addData("BL Current", BL.getCurrentPosition());
                telemetry.addData("FR Current", FR.getCurrentPosition());
                telemetry.addData("FL Current", FL.getCurrentPosition());

                telemetry.update();
            }
            // stop motors
            BR.setPower(0);
            BL.setPower(0);
            FR.setPower(0);
            FL.setPower(0);
        }
    }
}   // end class