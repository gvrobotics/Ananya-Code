package org.firstinspires.ftc.teamcode.Tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Shooter FF + P Test", group = "TEST")
public class PIDTuning extends OpMode {

    DcMotorEx fly1, fly2;
    Servo push1, push2, launch;

    // ===== TUNING VALUES =====
    double targetVelocity = 1300;   // ticks/sec
    double kF = 0.00052;
    double kP = 0.0026;

    boolean shooterOn = false;
    boolean prevRB = false;
    boolean pushOn = false;
    boolean prevA = false;
    private double pushUp1 = 0.7, pushDown1 = 0.4, pushUp2 = 0.25, pushDown2 = 0; //push 1 (R) - down is 0.4, up is 0.7 == push 2 (F) - down is 0, up is 0.25


    @Override
    public void init() {
        fly1 = hardwareMap.get(DcMotorEx.class, "f1");
        fly2 = hardwareMap.get(DcMotorEx.class, "f2");
        push1 = hardwareMap.get(Servo.class, "p1");
        push2 = hardwareMap.get(Servo.class, "p2");
        launch = hardwareMap.get(Servo.class, "l");

        fly1.setDirection(DcMotorSimple.Direction.REVERSE);
        fly2.setDirection(DcMotorSimple.Direction.REVERSE);
        push1.setDirection(Servo.Direction.REVERSE);
        push2.setDirection(Servo.Direction.FORWARD);
        launch.setDirection(Servo.Direction.FORWARD);

        // IMPORTANT: manual control
        fly1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        fly2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        fly1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        fly2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        push1.setPosition(pushDown1);
        push2.setPosition(pushDown2);
        launch.setPosition(0.3);
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
        boolean a = gamepad1.a;
        if (a && !prevA) {
            pushOn = !pushOn;
            push1.setPosition(pushOn ? pushUp1 : pushDown1); // up - down
            push2.setPosition(pushOn ? pushUp2 : pushDown2);
        }
        prevA = a;

        // ===== VELOCITY ADJUST =====
        if (gamepad1.dpadUpWasPressed()) targetVelocity += 20;
        if (gamepad1.dpadDownWasPressed()) targetVelocity -= 20;

        // ===== LAUNCH VELOCITY ADJUST =====
        if (gamepad1.dpadLeftWasPressed())
            launch.setPosition(launch.getPosition() + 0.05);

        if (gamepad1.dpadRightWasPressed())
            launch.setPosition(launch.getPosition() - 0.05);

        // ===== kF ADJUST =====
        if (gamepad2.dpadUpWasPressed()) kF = kF + 0.00001;
        if (gamepad2.dpadDownWasPressed()) kF = kF - 0.00001;

        // ===== kP ADJUST =====
        if (gamepad2.dpadLeftWasPressed()) kP += 0.0001;
        if (gamepad2.dpadRightWasPressed()) kP -= 0.0001;


        // ===== CONTROL LOOP =====
        double velocity = Math.abs(fly1.getVelocity()); // ONE encoder
        double error = targetVelocity - velocity;

        double power = 0;
        if (shooterOn) {
            power = (targetVelocity * kF) + (error * kP);
        }

        // power = Math.max(0, Math.min(1.0, power));

        fly1.setPower(power);
        fly2.setPower(power);

        // ===== TELEMETRY =====
        telemetry.addLine("=== SHOOTER FF + P TEST ===");
        telemetry.addData("Target Vel", targetVelocity);
        telemetry.addData("Fly Up", fly1.getVelocity());
        telemetry.addData("Fly Down", fly2.getVelocity());
        telemetry.addData("Launch", launch.getPosition());
        telemetry.addLine();
        telemetry.addData("kF", kF);
        telemetry.addData("kP", kP);
        telemetry.addData("Error", error);
        telemetry.addData("Power", power);

        telemetry.addData("Shooter", shooterOn ? "ON" : "OFF");
        telemetry.addData("Pusher", pushOn ? "UP" : "DOWN");
        telemetry.update();
    }
}
