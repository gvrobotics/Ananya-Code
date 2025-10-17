package org.firstinspires.ftc.teamcode; // Defines the package where this code belongs
// Import statements: brings in necessary FTC libraries
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp; // Annotation to mark this as a TeleOp mode
import com.qualcomm.robotcore.hardware.DcMotor; // Allows us to control DC motors
import com.qualcomm.robotcore.hardware.DcMotorSimple; // simple motor functions

@TeleOp // Marks this OpMode as a TeleOp, so it will appear on the driver station
public class Holo_Explain extends OpMode {
    // Declare motors for each wheel
    public DcMotor BR, BL, FR, FL; // BR = Back Right, BL = Back Left, FR = Front Right, FL = Front Left
    private double powerRX, powerLX, powerLY, // Right and Left joystick X and Y values
            robotAngle, // Angle of movement based on left joystick
            PowerMultiplier, // Magnitude of movement from left joystick
            lf, rb, rf, lb; // Power values for each motor


    @Override
    public void init() // Runs once when the Init button is pressed
    {
        // Map each motor variable to its configuration name on the driver station
        // So control hub knows what motor to run when
        BR = hardwareMap.get(DcMotor.class, "BR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        FL = hardwareMap.get(DcMotor.class, "FL");

        // Set motor directions
        // Some are reversed because of orientation
        BR.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set motors to brake when no power is applied
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize all motor powers to zero
        BR.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        FL.setPower(0);
    }


    @Override
    public void loop() // Runs repeatedly after play is pressed
    {
        // Read joystick inputs and scale them down by dividing by 1.5 for control
        powerLX = gamepad1.left_stick_x / 1.5; // Left joystick X-axis (strafe left/right)
        powerLY = gamepad1.left_stick_y / 1.5; // Left joystick Y-axis (forward/backward)
        powerRX = gamepad1.right_stick_x / 1.5; // Right joystick X-axis (rotation)

        // Calculate robot angle based on left joystick input
        robotAngle = Math.atan2(powerLX, powerLY); // atan2 returns the angle from X and Y inputs


        // Display telemetry (debug info) on driver station
        telemetry.addData("Robot angle:", robotAngle);
        telemetry.addData("powerRX: ", gamepad1.right_stick_x);
        telemetry.addData("powerRY: ", gamepad1.right_stick_y);
        telemetry.addData("powerLX: ", gamepad1.left_stick_x);
        telemetry.addData("powerLY: ", gamepad1.left_stick_y);

        telemetry.addData("BR: ", BR.getPower());
        telemetry.addData("BL: ", BL.getPower());
        telemetry.addData("FR: ", FR.getPower());
        telemetry.addData("FL: ", FL.getPower());
        telemetry.update();

        // Calculate the magnitude of the left joystick
        // Determines how fast the robot should move in that direction
        PowerMultiplier = Math.sqrt((Math.pow(powerLX, 2) + Math.pow(powerLY, 2)));

        // Calculate individual motor powers for a holonomic drive
        // Combines joystick movement and rotation
        //Pi/4 is used because each wheel is mounted at a 45 degree angle
        // Power RX is added/subtracted to each motor so the robot can spin while moving (+ is right, - is left)
        lf = (PowerMultiplier*(Math.sin(robotAngle+(Math.PI/4)))) - powerRX;
        rb = (PowerMultiplier*(Math.sin(robotAngle+(Math.PI/4)))) + powerRX;
        lb = (PowerMultiplier*-1*(Math.sin(robotAngle-(Math.PI/4)))) - powerRX;
        rf = (PowerMultiplier*-1*(Math.sin(robotAngle-(Math.PI/4)))) + powerRX;

        // Apply calculated powers to each motor
        BR.setPower(rb);
        BL.setPower(lb);
        FR.setPower(rf);
        FL.setPower(lf);

    }
}

