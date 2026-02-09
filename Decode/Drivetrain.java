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
        // ===== ODOMETRY CONFIGURATION =====
        odo.setOffsets(-4.33, -3.5, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );

        // Reset and set starting position
        odo.resetPosAndIMU();
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
        // MUST be called every loop for field-centric to work
        odo.update();

        if (gamepad.y && !prevToggle)
            fieldCentric = !fieldCentric;
        prevToggle = gamepad.y;

        double y = -gamepad.left_stick_y;
        double x = -gamepad.left_stick_x;
        double rx = gamepad.right_stick_x;

        rx += rotationOverride;

        if (fieldCentric) {
            Pose2D pos = odo.getPosition();
            double heading = pos.getHeading(AngleUnit.RADIANS);

            double rotationAngle = (0) - heading;
            double cosAngle = Math.cos(rotationAngle);
            double sinAngle = Math.sin(rotationAngle);

            // Rotate the movement vector by the robot's heading
            double forward = y * cosAngle + x * sinAngle;
            double strafe = -y * sinAngle + x * cosAngle;

            setPower(
                forward + strafe + rx,
                forward - strafe - rx,
                forward - strafe + rx,
                forward + strafe - rx);
        } else {
            setPower(
                y + x + rx,
                y - x - rx,
                y - x + rx,
                y + x - rx);
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
}
