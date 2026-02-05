package org.firstinspires.ftc.teamcode.Tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "New Shooter Test", group = "B")
public class ShooterFPTest extends OpMode {

    DcMotorEx fly1, fly2;
    Servo push1, push2;

    // ===== TUNING VALUES =====
    double targetVelocity = 1500; // ticks/sec
    double kF = 1/2320;
    double kP = 0.0005; // START LOW use dpad left and right to increment

    boolean shooterOn = false;
    boolean prevRB = false;
    boolean pushOn = false;
    boolean prevA = false, currA = false;

    @Override
    public void init() {
        fly1 = hardwareMap.get(DcMotorEx.class, "f1");
        fly2 = hardwareMap.get(DcMotorEx.class, "f2");
        push1 = hardwareMap.get(Servo.class, "p1");
        push2 = hardwareMap.get(Servo.class, "p2");

        fly1.setDirection(DcMotorSimple.Direction.REVERSE);
        fly2.setDirection(DcMotorSimple.Direction.REVERSE);
        push1.setDirection(Servo.Direction.REVERSE);
        push2.setDirection(Servo.Direction.FORWARD);


        // IMPORTANT: manual control
        fly1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        fly2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        fly1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        fly2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        push1.setPosition(0.4); //push 1 (R) - down is 0.4, up is 0.7 == push 2 (F) - down is 0, up is 0.25
        push2.setPosition(0);
    }

    @Override
    public void loop() {

        // ===== SHOOTER TOGGLE =====
        boolean rb = gamepad1.right_bumper;
        if (rb && !prevRB) {
            shooterOn = !shooterOn;
        }
        prevRB = rb;

        // ===== PUSH TOGGLE =====
        prevA = currA;
        currA = gamepad1.a;
        if (currA && !prevA) {
            pushOn = !pushOn;
            if (pushOn) {
                push1.setPosition(0.7);
                push2.setPosition(0.25);
            } else {
                push1.setPosition(0.4);
                push2.setPosition(0);
            }
        }

        // ===== TARGET VELOCITY =====
        if (gamepad1.dpadUpWasPressed()) targetVelocity += 10;
        if (gamepad1.dpadDownWasPressed()) targetVelocity -= 10;

        if (gamepad1.dpadLeftWasPressed()) kP -= 0.001;
        if (gamepad1.dpadRightWasPressed()) kP += 0.001;

        // ===== LOOP =====
        double velocity = Math.abs(fly1.getVelocity()); // only ONE encoder
        double error = targetVelocity - velocity;

        double power = 0;
        if (shooterOn) {
            power = (targetVelocity * kF) + (error * kP);
        }

        // add in at the end to cap power
        //power = Math.max(0, Math.min(1.0, power));

        fly1.setPower(power);
        fly2.setPower(power);

        // ===== TELEMETRY =====
        telemetry.addData("Shooter", shooterOn ? "ON" : "OFF");
        telemetry.addData("Target Vel", targetVelocity);
        telemetry.addData("Actual Vel 1", fly1.getVelocity());
        telemetry.addData("Actual Vel 2", fly2.getVelocity());
        telemetry.addData("Error", error);
        telemetry.addData("kF", kF);
        telemetry.addData("kP", kP);
        telemetry.addData("Power", power);
        telemetry.addLine("========PUSH========");
        telemetry.addData("Pusher", pushOn ? "UP" : "DOWN");
        telemetry.addData("Push1 pos", push1.getPosition());
        telemetry.addData("Push2 pos", push2.getPosition());
        telemetry.update();
    }
}
