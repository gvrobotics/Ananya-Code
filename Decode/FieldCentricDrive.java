package org.firstinspires.ftc.teamcode.Odo;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp
public class FieldCentricDrive extends OpMode {

    public DcMotor BR, BL, FR, FL, spin1, spin2;
    public Servo flip;
    GoBildaPinpointDriver odo;
    boolean motorOn;
    private ElapsedTime timer = new ElapsedTime();
    private Boolean t = false;


    @Override
    public void init()
    {
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        spin1 = hardwareMap.get(DcMotor.class, "s1");
        spin2 = hardwareMap.get(DcMotor.class, "s2");
        flip = hardwareMap.get(Servo.class, "f");
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");


        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        spin1.setDirection(DcMotorSimple.Direction.FORWARD);
        spin2.setDirection(DcMotorSimple.Direction.REVERSE);
        flip.setDirection(Servo.Direction.REVERSE);

        //Odometry Computer Configuration
        odo.setOffsets(128, 128, DistanceUnit.MM); //where the odometry computer is on the robot
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        //Odometry Starting Position
        odo.resetPosAndIMU();
        Pose2D startingPosition = new Pose2D(DistanceUnit.MM, -923.925, 1601.47, AngleUnit.RADIANS, 0); //where we start and where heading is according to the field
        odo.setPosition(startingPosition);

        telemetry.addData("Status: ", "Initialized");
        telemetry.addData("X offset: ", odo.getXOffset(DistanceUnit.MM));
        telemetry.addData("Y offset: ", odo.getYOffset(DistanceUnit.MM));
        telemetry.addData("Device Version Number: ", odo.getDeviceVersion());
        telemetry.addData("Device Scalar: ", odo.getYawScalar());
        telemetry.update();
    }


    public void moveRobot() {
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        Pose2D pos = odo.getPosition();
        double heading = pos.getHeading(AngleUnit.RADIANS);

        double cosAngle = Math.cos((Math.PI / 2) - heading);
        double sinAngle = Math.sin((Math.PI / 2) - heading);

        double globalStrafe = -forward * sinAngle + strafe * cosAngle;
        double globalForward = forward * cosAngle + strafe * sinAngle;

        double br, bl, fr, fl;

        fl = globalForward + globalStrafe + rotate;
        fr = globalForward - globalStrafe - rotate;
        bl = globalForward - globalStrafe + rotate;
        br = globalForward + globalStrafe - rotate;

        BR.setPower(br);
        BL.setPower(bl);
        FR.setPower(fr);
        FL.setPower(fl);

        telemetry.addData("Robot XPos: ", pos.getX(DistanceUnit.MM));
        telemetry.addData("Robot YPos: ", pos.getY(DistanceUnit.MM));
        telemetry.addData("Robot Heading: ", heading);
        telemetry.addData("Forward Speed: ", globalForward);
        telemetry.addData("Strafe Speed: ", globalStrafe);
        telemetry.update();
    }

    @Override
    public void loop() {
        moveRobot();

        Pose2D pos = odo.getPosition();
        telemetry.addData("Robot X Pos: ", odo.getPosX(DistanceUnit.MM));
        telemetry.addData("Robot Y: ", odo.getPosY(DistanceUnit.MM));
        telemetry.addData("Robot X Angle: ", pos.getHeading(AngleUnit.DEGREES));
        telemetry.update();

        // ============================ Intake/Outtake ============================
        if (gamepad1.x) {
            // Set to 0.7 when X is pressed
            flip.setPosition(0.7);
            t = false;
        }
        else if (gamepad1.y && !t) {
            // When Y is pressed, flip down
            flip.setPosition(0.7);
            timer.reset();
            t = true;
        }
        else if (t && !gamepad1.y && timer.seconds() > 0.2) {
            // After Y is released for 0.2s, flip back up
            flip.setPosition(0.85);
            t = false;
        }

        // Shooter
        // When bumpers pressed
        if (gamepad1.left_bumper) {
            motorOn = !motorOn;  // toggle the state (true -> false, false -> true)
        }
        // Run or stop the motor depending on the current state
        if (motorOn) { //on
            spin1.setPower(0.4);
            spin2.setPower(0.4);
        }
        if (!motorOn) { //off
            spin1.setPower(0.0);
            spin2.setPower(0.0);
        }

        telemetry.addData("BR: ", BR.getPower());
        telemetry.addData("BL: ", BL.getPower());
        telemetry.addData("FR: ", FR.getPower());
        telemetry.addData("FL: ", FL.getPower());
        telemetry.addData("spin1: ", spin1.getPower());
        telemetry.addData("spin2: ", spin2.getPower());
        telemetry.addData("flip: ", flip.getPosition());
        telemetry.addData("Outtake On: ", motorOn);
        telemetry.update();

        odo.update();
    }
}
