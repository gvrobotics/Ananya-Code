package org.firstinspires.ftc.teamcode.Tele;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Odo.GoBildaPinpointDriver;

public class Drivetrain {

    DcMotor BR, BL, FR, FL;
    GoBildaPinpointDriver odo;

    boolean fieldCentric = true;
    boolean prevToggle;

    public Drivetrain(HardwareMap hw) {
        BR = hw.get(DcMotor.class, "BR");
        BL = hw.get(DcMotor.class, "BL");
        FR = hw.get(DcMotor.class, "FR");
        FL = hw.get(DcMotor.class, "FL");

        odo = hw.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-4.33, -3.5, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);

        odo.resetPosAndIMU();
        //Pose2D startingPosition = new Pose2D();
        Pose2D startingPosition = new Pose2D(DistanceUnit.MM, -923.925, 1601.47, AngleUnit.RADIANS, 0);
        odo.setPosition(startingPosition);

        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);

        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BR.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        FL.setPower(0);
    }

    public void update(Gamepad gamepad, double rotationOverride) {
        odo.update();

        if (gamepad.y && !prevToggle)
            fieldCentric = !fieldCentric;
        prevToggle = gamepad.y;

        double forward = -gamepad.left_stick_y * 1.2;
        double strafe = gamepad.left_stick_x * 1.4;
        double rotate = gamepad.right_stick_x * 1.2;

        rotate += rotationOverride;

        if (fieldCentric) {
            Pose2D pos = odo.getPosition();
            double heading = pos.getHeading(AngleUnit.RADIANS);

            double rotationAngle = 0 - heading;
            double cosAngle = Math.cos(rotationAngle);
            double sinAngle = Math.sin(rotationAngle);

            double globalstrafe = -forward * sinAngle + strafe * cosAngle;
            double globalforward = forward * cosAngle + strafe * sinAngle;

            // Calculate powers
            double fl = globalforward + globalstrafe + rotate;
            double fr = globalforward - globalstrafe - rotate;
            double bl = globalforward - globalstrafe + rotate;
            double br = globalforward + globalstrafe - rotate;

            setPower(fl, fr, bl, br);

        } else {
            // Calculate powers
            double fl = forward + strafe + rotate;
            double fr = forward - strafe - rotate;
            double bl = forward - strafe + rotate;
            double br = forward + strafe - rotate;

            setPower(fl, fr, bl, br);
        }
    }

    public void update(Gamepad gamepad1) {
        update(gamepad1, 0);
    }

    public double getBR() { return BR.getPower(); }
    public double getBL() { return BL.getPower(); }
    public double getFR() { return FR.getPower(); }
    public double getFL() { return FL.getPower(); }
    public boolean isFieldCentric() {
        return fieldCentric;
    }
    public Pose2D getPose() {
        return odo.getPosition();
    }

    private void setPower(double fl, double fr, double bl, double br) {
        FL.setPower(fl);
        FR.setPower(fr);
        BL.setPower(bl);
        BR.setPower(br);
    }

    public void autoMove(double forward, double strafe, double rotation) {
        // Standard mecanum drive calculations
        double frontLeftPower  = forward + strafe + rotation;
        double frontRightPower = forward - strafe - rotation;
        double backLeftPower   = forward - strafe + rotation;
        double backRightPower  = forward + strafe - rotation;

        // Set motor powers (adjust motor names to match your hardware config)
        FL.setPower(frontLeftPower);
        FR.setPower(frontRightPower);
        BL.setPower(backLeftPower);
        BR.setPower(backRightPower);
    }
}