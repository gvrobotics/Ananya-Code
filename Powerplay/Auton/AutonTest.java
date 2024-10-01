package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutonTest extends LinearOpMode {
    public DcMotor BR, BL, FR, FL, linear;
    public Servo claw;
    double power = 0.5;

    public enum robotMotion {
        forward, backward, right, left, openClaw, closeClaw, linegrup, lineardown;
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        linear = hardwareMap.get(DcMotor.class, "L");
        claw = hardwareMap.get(Servo.class, "c");

        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        linear.setDirection(DcMotorSimple.Direction.REVERSE);

        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
        linear.setPower(0);
        claw.setPosition(0);

        waitForStart();
        movementRL(300, robotMotion.right, 0.5);
        movementRL(500, robotMotion.left, 0.5);
        movementFB(200, robotMotion.left, 0.5);
        Clawmovement(750, robotMotion.closeClaw, 0.45);
        linearup(750,0.6);
        lineardown(600,0.8);
        robotsleep(200,0.00);
    }

    private void movementFB(int Sleep, robotMotion action, double power) {
        if (action == robotMotion.forward) {
            BR.setPower(power);
            FR.setPower(power);
            BL.setPower(power);
            FL.setPower(power);
            sleep(Sleep);
            BR.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            FL.setPower(0);
        } else if (action == robotMotion.backward) {
            BR.setPower(power);
            FR.setPower(power);
            BL.setPower(power);
            FL.setPower(power);
            sleep(Sleep);
            BR.setPower(0);
            FR.setPower(0);
            BL.setPower(0);
            FL.setPower(0);
        }
    }
        private void movementRL(int Sleep, robotMotion action, double power)
        {
            if (action == robotMotion.left)
            {
                BR.setPower(power);
                FR.setPower(power);
                BL.setPower(-power);
                FL.setPower(-power);
                sleep(Sleep);
                BR.setPower(0);
                FR.setPower(0);
                BL.setPower(0);
                FL.setPower(0);
            } else if (action == robotMotion.right)
            {
                BR.setPower(-power);
                FR.setPower(-power);
                BL.setPower(power);
                FL.setPower(power);
                sleep(Sleep);
                BR.setPower(0);
                FR.setPower(0);
                BL.setPower(0);
                FL.setPower(0);
            }
        }
        private void linearup ( int Sleep, double power)
        {
            linear.setPower(power);
            sleep(Sleep);
            linear.setPower(0);
        }
        private void lineardown (int Sleep, double power)
        {
            linear.setPower(power);
            sleep(Sleep);
            linear.setPower(0);
        }

        private void Clawmovement (int Sleep, robotMotion action, double position)
        {
            if(action == robotMotion.openClaw)
            {
                claw.setPosition(position);
                sleep(Sleep);
            }
            else if(action == robotMotion.closeClaw) {
                claw.setPosition(position);
                sleep(Sleep);
            }
        }

    private void robotsleep(int sleep, double power)
    {
        BR.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        FL.setPower(0);
    }
}
