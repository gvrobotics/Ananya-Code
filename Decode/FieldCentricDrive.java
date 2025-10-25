package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
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
        odo.setOffsets(128, 128); //where the odometry computer is on the robot
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        //Odometry Starting Position
        odo.resetPosAndIMU();
        Pose2D startingPosition = new Pose2D(DistanceUnit.MM, -923.925, 1601.47, AngleUnit.RADIANS, 0); //where we start and where heading is according to the field
        odo.setPosition(startingPosition);

        telemetry.addData("Status", "Initialized");
        //telemetry.addData("X offset", odo.getXOffset());
        //telemetry.addData("Y offset", odo.getYOffset());
        telemetry.addData("Device Version Number", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
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



    }
    @Override
    public void loop() {

    }


}
