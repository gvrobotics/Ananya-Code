package org.firstinspires.ftc.teamcode.Odo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp (name = "Field Centric Drive", group = "Odo")
public class FieldCentricDrive extends OpMode {
    public DcMotor BR, BL, FR, FL;
    GoBildaPinpointDriver odo;

    @Override
    public void init() {
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // Set motor directions so forward command makes robot move forward
        // Adjust these if your robot moves backward when you push stick forward
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);

        // Configure the odometry computer
        // setOffsets: where the odometry pod is mounted relative to robot center (in inches)
        odo.setOffsets(-3, 3, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        // Set encoder directions (REVERSED means counting down when moving forward)
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();

        // Set the starting position on the field
        // This tells the robot where it is at the start of the match
        // X: -923.925mm, Y: 1601.47mm, Heading: 0 radians (facing field's "forward")
        Pose2D startingPosition = new Pose2D(DistanceUnit.MM, -923.925, 1601.47, AngleUnit.RADIANS, 0);
        odo.setPosition(startingPosition);

        // Display initialization info to driver station
        telemetry.addData("Status: ", "Initialized");
        telemetry.addData("X offset: ", odo.getXOffset(DistanceUnit.MM));
        telemetry.addData("Y offset: ", odo.getYOffset(DistanceUnit.MM));
        telemetry.addData("Device Version Number: ", odo.getDeviceVersion());
        telemetry.addData("Device Scalar: ", odo.getYawScalar());
        telemetry.update();
    }

    @Override
    public void loop() {
        odo.update();

        // Calculate and apply motor powers based on joystick input
        moveRobot();

        // Get current robot position for telemetry
        Pose2D pos = odo.getPosition();
        telemetry.addData("Robot X Pos: ", odo.getPosX(DistanceUnit.MM));
        telemetry.addData("Robot Y: ", odo.getPosY(DistanceUnit.MM));
        telemetry.addData("Robot X Angle: ", pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("BR: ", BR.getPower());
        telemetry.addData("BL: ", BL.getPower());
        telemetry.addData("FR: ", FR.getPower());
        telemetry.addData("FL: ", FL.getPower());
        telemetry.update();
    }

    public void moveRobot() {
        // STEP 1: Read joystick inputs
        // Get driver inputs from gamepad (values range from -1.0 to 1.0)
        double forward = -gamepad1.left_stick_y / 1.2;  // negative because Y axis is inverted on gamepads
        double strafe = gamepad1.left_stick_x / 1.2;    // positive = right, negative = left
        double rotate = gamepad1.right_stick_x / 1.2;   // positive = clockwise, negative = counterclockwise

        // STEP 2: Get robot's current heading
        Pose2D pos = odo.getPosition();
        double heading = pos.getHeading(AngleUnit.RADIANS);  // Robot's angle on the field

        // STEP 3: Field-centric transformation

        // We rotate the driver's input by the robot's heading so "forward" on the joystick
        // always means "forward relative to the field" not "forward relative to the robot"
        // IMPORTANT: To change which direction is "field forward", modify this angle offset
        double rotationAngle = (Math.PI / 2) - heading;
        double cosAngle = Math.cos(rotationAngle);
        double sinAngle = Math.sin(rotationAngle);

        // Apply rotation matrix to transform robot-centric to field-centric
        // This math rotates the forward/strafe vector by the robot's heading
        double globalStrafe = -forward * sinAngle + strafe * cosAngle;
        double globalForward = forward * cosAngle + strafe * sinAngle;

        // STEP 4: Calculate mecanum wheel powers
        // Standard mecanum drive equations
        double fl = globalForward + globalStrafe + rotate;
        double fr = globalForward - globalStrafe - rotate;
        double bl = globalForward - globalStrafe + rotate;
        double br = globalForward + globalStrafe - rotate;

        // STEP 5: Normalize wheel powers
        // Find the largest absolute power
        double max = Math.max(
                Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))
        );

        // If any power exceeds 1.0, scale all powers down proportionally
        // This maintains the direction while keeping powers in valid range
        if (max > 1.0) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }

        // STEP 6: Send powers to motors
        BR.setPower(br);
        BL.setPower(bl);
        FR.setPower(fr);
        FL.setPower(fl);

        telemetry.addData("Robot XPos: ", pos.getX(DistanceUnit.MM));
        telemetry.addData("Robot YPos: ", pos.getY(DistanceUnit.MM));
        telemetry.addData("Robot Heading: ", heading);
        telemetry.addData("Forward Speed: ", globalForward);
        telemetry.addData("Strafe Speed: ", globalStrafe);
    }
}

// Replace (Math.PI / 2) with:
//     0           -> Field forward = 0° (red alliance wall in FTC)
//     Math.PI/2   -> Field forward = 90° (current setting)
//     Math.PI     -> Field forward = 180° (blue alliance wall)
//     3*Math.PI/2 -> Field forward = 270°
// Or use any angle in radians for custom orientations!
