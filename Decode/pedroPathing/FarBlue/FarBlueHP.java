package org.firstinspires.ftc.teamcode.pedroPathing.FarBlue;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Classes.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Classes.Flywheel;

@Autonomous(group = "Far Blue", name = "1. Far Blue HP", preselectTeleOp = "Tele 5440")
public class FarBlueHP extends OpMode {

    private TelemetryManager panelsTelemetry;
    private Follower follower;
    private Flywheel shooter = new Flywheel();

    // Timers
    private Timer pathTimer, opModeTimer;

    // State flags
    private boolean pathStarted = false, shotsTriggered = false, alignmentChecked = false;

    // Shooting parameters
    private static final double SHOOTING_VELOCITY = 1150, SHOOTING_ANGLE = 0.15;
    private boolean USE_AUTO_ADJUST = true;

    public enum PathState {
        DRIVE_STARTPOS_SHOOT_POS,
        ALIGN_AND_SHOOT_PRELOAD,
        DRIVE_SHOOTPOS_HP1_START,
        DRIVE_HP1_START_TO_END,
        DRIVE_HP1_END_TO_HP2_START,
        DRIVE_HP2_START_TO_END,
        DRIVE_HP2_END_TO_SHOOTPOS,
        ALIGN_AND_SHOOT_HP,
        DRIVE_SHOOT_ENDPOS,
        COMPLETE
    }

    private PathState pathState;

    // ===== POSES =====
    private final Pose startPose = new Pose(84.6, 9, Math.toRadians(90)).mirror();
    private final Pose shootPose = new Pose(82.5, 21.1, Math.toRadians(65)).mirror();
    private final Pose hp1Start = new Pose(108.8, 16.5, Math.toRadians(0)).mirror();
    private final Pose hp1End = new Pose(134.9, 12.2, Math.toRadians(-19)).mirror();
    private final Pose hp2Start = new Pose(112.1, 7.5, Math.toRadians(0)).mirror();
    private final Pose hp2End = new Pose(136, 7.5, Math.toRadians(0)).mirror();
    private final Pose endPose = new Pose(108.5, 12.2, Math.toRadians(0)).mirror();

    // ===== PATHS =====
    private PathChain driveStartPosShootPos, driveShootPosHp1StartPose,
            driveHp1StartToEndPose, driveHp1EndToHp2StartPose,
            driveHp2StartToEndPose, driveHp1ToShootPose, driveShootPosEndPose;

    public void buildPaths() {
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        driveShootPosHp1StartPose = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, hp1Start))
                .setLinearHeadingInterpolation(shootPose.getHeading(), hp1Start.getHeading())
                .setTimeoutConstraint(1000)
                .build();

        driveHp1StartToEndPose = follower.pathBuilder()
                .addPath(new BezierLine(hp1Start, hp1End))
                .setLinearHeadingInterpolation(hp1Start.getHeading(), hp1End.getHeading())
                .build();

//        driveHp1EndToHp2StartPose = follower.pathBuilder()
//                .addPath(new BezierLine(hp1End, hp2Start))
//                .setLinearHeadingInterpolation(hp1End.getHeading(), hp2Start.getHeading())
//                .build();
//
//        driveHp2StartToEndPose = follower.pathBuilder()
//                .addPath(new BezierLine(hp2Start, hp2End))
//                .setLinearHeadingInterpolation(hp2Start.getHeading(), hp2End.getHeading())
//                .setTimeoutConstraint(300)
//                .build();

        driveHp1ToShootPose = follower.pathBuilder()
                .addPath(new BezierLine(hp1End, shootPose))
                .setLinearHeadingInterpolation(hp1End.getHeading(), shootPose.getHeading())
                .build();

        driveShootPosEndPose = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
                .build();
    }

    public void statePathUpdate() {

        switch (pathState) {

            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos, true);
                setPathState(PathState.ALIGN_AND_SHOOT_PRELOAD);
                break;

            case ALIGN_AND_SHOOT_PRELOAD:
                if (!follower.isBusy()) {
                    handleShoot(PathState.DRIVE_SHOOTPOS_HP1_START);
                }
                break;

            // ===== HP1 COLLECTION =====

            case DRIVE_SHOOTPOS_HP1_START:
                handleDriveToRowStart(driveShootPosHp1StartPose, PathState.DRIVE_HP1_START_TO_END);
                break;

            case DRIVE_HP1_START_TO_END:
                handleRowCollection(driveHp1StartToEndPose, PathState.DRIVE_HP1_END_TO_HP2_START);
                break;

            case DRIVE_HP1_END_TO_HP2_START:
                handleDriveToRowStart(driveHp1ToShootPose, PathState.ALIGN_AND_SHOOT_HP);
                break;

            case ALIGN_AND_SHOOT_HP:
                if (!follower.isBusy()) {
                    handleShoot(PathState.DRIVE_SHOOT_ENDPOS);
                }
                break;

            case DRIVE_SHOOT_ENDPOS:
                if (!pathStarted) {
                    follower.followPath(driveShootPosEndPose, true);
                    pathStarted = true;
                }

                if (!follower.isBusy()) {
                    setPathState(PathState.COMPLETE);
                }
                break;

            case COMPLETE:
                telemetry.addLine("Auto Complete");
                break;
        }
    }

    // ============== HELPER METHOD: Drive to Row Start ==============
    private void handleDriveToRowStart(PathChain path, PathState nextState) {
        if (!pathStarted) {
            follower.followPath(path, true);
            pathStarted = true;
        }

        if (!follower.isBusy()) {
            setPathState(nextState);
        }
    }

    // ============== HELPER METHOD: Row Collection ==============
    private void handleRowCollection(PathChain path, PathState nextState) {
        if (!pathStarted) {
            follower.followPath(path, true);
            shooter.setIntakePower(0.7);
            pathStarted = true;
        }

        if (!follower.isBusy()) {
            shooter.stopIntake();
            setPathState(nextState);
        }
    }

    // ============== HELPER METHOD: Shoot ==============
    private void handleShoot(PathState nextState) {
        if (!alignmentChecked) {
            setupShot();
            alignmentChecked = true;
        }

        if (!shotsTriggered) {
            shooter.fireShots(3);
            shotsTriggered = true;
        } else if (!shooter.isBusy()) {
            setPathState(nextState);
        }
    }

    private void setupShot() {
        shooter.setAutoAdjust(USE_AUTO_ADJUST);

        // Always set fallback parameters in case Limelight fails
        shooter.setShootingParameters(SHOOTING_VELOCITY, SHOOTING_ANGLE);
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
        pathStarted = false;
        shotsTriggered = false;
        alignmentChecked = false;
    }

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        shooter.init(hardwareMap);

        buildPaths();
        follower.setPose(startPose);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {

        follower.update();
        shooter.update();
        statePathUpdate();

        panelsTelemetry.debug("=== PATH ===");
        panelsTelemetry.debug("State", pathState.toString());
        panelsTelemetry.debug("Time", pathTimer.getElapsedTimeSeconds());

        panelsTelemetry.debug("=== POSITION ===");
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));

        panelsTelemetry.debug("=== SHOOTER ===");
        panelsTelemetry.debug("State", shooter.getState());
        panelsTelemetry.debug("Shots Left", shooter.getShotsRemaining());
        panelsTelemetry.debug("Flywheel RPM", shooter.getFlywheelRPM());

        panelsTelemetry.update(telemetry);
    }
}