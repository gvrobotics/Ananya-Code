package org.firstinspires.ftc.teamcode.Tele;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class FlywheelPIDTemplate extends OpMode {
    public DcMotorEx flywheelMotor;
    public double highVelocity = 1500;
    public double lowVelocity = 500;
    double currTargetVelocity = highVelocity;
    double P, F = 0;

    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};
    int stepIndex = 1;

    @Override
    public void init() {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "f1");
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pidfCoeffs = new PIDFCoefficients(P, 0, 0, F);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoeffs);
        telemetry.addLine("Init Complete");
    }

    @Override
    public void loop() {
        // get all gamepad commands
        // set target velocity
        // update telemetry

        if (gamepad1.yWasPressed()) {
            if (currTargetVelocity == highVelocity) {
                currTargetVelocity = lowVelocity;
            } else {
                currTargetVelocity = highVelocity;
            }

        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }

        if (gamepad1.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }
        if (gamepad1.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        }

        // set new PIDF Coefficients
        PIDFCoefficients pidfCoeffs = new PIDFCoefficients(P, 0, 0, F);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoeffs);

        // set velocity
        flywheelMotor.setVelocity(currTargetVelocity);

        double currVelocity = flywheelMotor.getVelocity();
        double error = currTargetVelocity - currVelocity;

        telemetry.addData("Target Velocity", currTargetVelocity);
        telemetry.addData("Current Velocity", "%.2f", currVelocity);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addLine("--------------------------------------------");
        telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", P);
        telemetry.addData("Tuning F", "%.4f (D-Pad R/L)", F);
        telemetry.addData("Step Size", "%.4f (B)", stepSizes[stepIndex]);
        telemetry.update();
        }
    }
}
