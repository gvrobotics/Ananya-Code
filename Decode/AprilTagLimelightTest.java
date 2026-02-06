package org.firstinspires.ftc.teamcode.AprilTags;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Odo.GoBildaPinpointDriver;

public class AprilTagLimelightTest extends OpMode {
    private Limelight3A limelight;
    private IMU imu;
    GoBildaPinpointDriver odo;

    public void init () {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        limelight.pipelineSwitch(0); //TODO: Change pipeline

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

        // ===== ODOMETRY CONFIGURATION =====
        odo.setOffsets(-4.33, -3.5, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        // Reset and set starting position
        odo.resetPosAndIMU();
        Pose2D startingPosition = new Pose2D(DistanceUnit.MM, -923.925, 1601.47, AngleUnit.RADIANS, 0);
        odo.setPosition(startingPosition);
    }

    public void start() {
        limelight.start();
    }

    public void loop () {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());  //odo.getHeading(); - for odometry
        LLResult llresult = limelight.getLatestResult();
        if (llresult != null && llresult.isValid()) {
            Pose3D botPose = llresult.getBotpose_MT2();
            //Pose2D botPose = llresult.getBotpose(); - for odometry
            double dist = getDistanceFromTag(llresult.getTa());
            telemetry.addData("Distance", dist);
            telemetry.addData("Tx", llresult.getTx());
            telemetry.addData("Ty", llresult.getTy());
            telemetry.addData("Target Area", llresult.getTa());
            telemetry.addData("Bot Pose", botPose.toString());
        }
    }

    public double getDistanceFromTag (double ta) {
        double scale = 0; // TODO: Plug in slope from curved line
        double distance = (scale / ta);
        return distance;
    }
}
