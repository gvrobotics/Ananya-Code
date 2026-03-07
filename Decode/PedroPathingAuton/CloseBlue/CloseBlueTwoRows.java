package org.firstinspires.ftc.teamcode.PedroPathingAuton.CloseBlue;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.PedroPathingAuton.Classes.Constants;
import org.firstinspires.ftc.teamcode.PedroPathingAuton.Classes.Flywheel;

@Autonomous(group = "Close Blue", name = "2. Close Blue Two Rows", preselectTeleOp = "Tele 5440")
public class CloseBlueTwoRows extends OpMode {
    private TelemetryManager panelsTelemetry;
    private Follower follower;
    private Flywheel shooter = new Flywheel();

    // Timers
    private Timer pathTimer, opModeTimer;

    // States
    private boolean pathStarted = false, shotsTriggered = false, alignmentChecked = false;

    // Shooting parameters (just in case auto aim doesn't work)
    private static final double SHOOTING_VELOCITY = 880, SHOOTING_ANGLE = 0.67;
    private boolean USE_AUTO_ADJUST = true; // Set to false for manual parameters

    public enum PathState {
        DRIVE_STARTPOS_SHOOT_POS,
        ALIGN_AND_SHOOT_PRELOAD,
        //------FIRST ROW------
        DRIVE_SHOOTPOS_FIRSTROW_START,
        DRIVE_FIRSTROW_START_TO_END,
        DRIVE_FIRST_ENDPOS_SHOOTPOS,
        ALIGN_AND_SHOOT_FIRSTROW,
        //-----SECOND ROW-------
        DRIVE_SHOOT_TO_SECONDROW_START,
        DRIVE_SECONDROW_START_TO_END,
        DRIVE_SECOND_ENDPOS_SHOOTPOS,
        ALIGN_AND_SHOOT_SECONDROW,
        SHOOT_TO_ENDPOS,
        COMPLETE
    }
    PathState pathState;
    private double rowStart = 99, rowEnd = 132.5;

    // Poses
    private final Pose startPose = new Pose(117.5, 129.2, Math.toRadians(45)).mirror();
    private final Pose shootPose = new Pose(92.6, 99.3, Math.toRadians(40)).mirror();
    private final Pose firstRowStart = new Pose(rowStart, 83.6, Math.toRadians(0)).mirror();
    private final Pose firstRowEnd = new Pose(128, 83.6, Math.toRadians(0)).mirror();
    private final Pose secondRowStart = new Pose(rowStart, 59.5, Math.toRadians(0)).mirror();
    private final Pose move = new Pose(123, 59.6, Math.toRadians(0)).mirror();
    private final Pose secondRowEnd = new Pose(rowEnd, 59.5, Math.toRadians(0)).mirror();
    private final Pose endPose = new Pose(95, 121.3, Math.toRadians(0)).mirror();

    // Paths
    private PathChain driveStartPosShootPos, driveShootPosFirstRowStartPose,
            driveFirstRowStartToEndPose, driveFirstRowtoShootPose,
            driveShootPosSecondRowStartPose, driveSecondRowStartToEndPose, driveSecondRowToShootPose,
            driveShootPosEndPose;

    public void buildPaths() {
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        //---------------------FIRST ROW START----------------------
        driveShootPosFirstRowStartPose = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, firstRowStart))
                .setLinearHeadingInterpolation(shootPose.getHeading(), firstRowStart.getHeading())
                .build();

        driveFirstRowStartToEndPose = follower.pathBuilder()
                .addPath(new BezierLine(firstRowStart, firstRowEnd))
                .setLinearHeadingInterpolation(firstRowStart.getHeading(), firstRowEnd.getHeading())
                .build();

        driveFirstRowtoShootPose = follower.pathBuilder()
                .addPath(new BezierLine(firstRowEnd, shootPose))
                .setLinearHeadingInterpolation(firstRowEnd.getHeading(), shootPose.getHeading())
                .build();

        //---------------------SECOND ROW START------------------------
        driveShootPosSecondRowStartPose = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, secondRowStart))
                .setLinearHeadingInterpolation(shootPose.getHeading(), secondRowStart.getHeading())
                .build();

        driveSecondRowStartToEndPose = follower.pathBuilder()
                .addPath(new BezierLine(secondRowStart, secondRowEnd))
                .setLinearHeadingInterpolation(secondRowStart.getHeading(), secondRowEnd.getHeading())
                .build();

        driveSecondRowToShootPose = follower.pathBuilder()
                .addPath(new BezierLine(secondRowEnd, move))
                .setLinearHeadingInterpolation(secondRowEnd.getHeading(), move.getHeading())
                .addPath(new BezierLine(move, shootPose))
                .setLinearHeadingInterpolation(move.getHeading(), shootPose.getHeading())
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
                    handleShoot(PathState.DRIVE_SHOOTPOS_FIRSTROW_START);
                }
                break;

            // ==================== FIRST ROW ====================
            case DRIVE_SHOOTPOS_FIRSTROW_START:
                handleDriveToRowStart(driveShootPosFirstRowStartPose, PathState.DRIVE_FIRSTROW_START_TO_END);
                break;

            case DRIVE_FIRSTROW_START_TO_END:
                handleRowCollection(driveFirstRowStartToEndPose, PathState.DRIVE_FIRST_ENDPOS_SHOOTPOS);
                break;

            case DRIVE_FIRST_ENDPOS_SHOOTPOS:
                follower.followPath(driveFirstRowtoShootPose, true);
                setPathState(PathState.ALIGN_AND_SHOOT_FIRSTROW);
                break;

            case ALIGN_AND_SHOOT_FIRSTROW:
                if (!follower.isBusy()) {
                    handleShoot(PathState.DRIVE_SHOOT_TO_SECONDROW_START);
                }
                break;

            // ==================== SECOND ROW ====================
            case DRIVE_SHOOT_TO_SECONDROW_START:
                handleDriveToRowStart(driveShootPosSecondRowStartPose, PathState.DRIVE_SECONDROW_START_TO_END);
                break;

            case DRIVE_SECONDROW_START_TO_END:
                handleRowCollection(driveSecondRowStartToEndPose, PathState.DRIVE_SECOND_ENDPOS_SHOOTPOS);
                break;

            case DRIVE_SECOND_ENDPOS_SHOOTPOS:
                follower.followPath(driveSecondRowToShootPose, true);
                setPathState(PathState.ALIGN_AND_SHOOT_SECONDROW);
                break;

            case ALIGN_AND_SHOOT_SECONDROW:
                if (!follower.isBusy()) {
                    handleShoot(PathState.SHOOT_TO_ENDPOS);
                }
                break;

            case SHOOT_TO_ENDPOS:
                if (!follower.isBusy()) {
                    follower.followPath(driveShootPosEndPose, true);
                    setPathState(PathState.COMPLETE);
                }
                break;

            case COMPLETE:
                telemetry.addLine("YAY DONE");
                break;

            default:
                telemetry.addLine("ERROR: Invalid State");
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
        if (!shooter.hasValidTarget()) {
            shooter.setShootingParameters(SHOOTING_VELOCITY, SHOOTING_ANGLE);
        }
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
        panelsTelemetry.debug("Auto-Adjust", USE_AUTO_ADJUST ? "ENABLED" : "DISABLED");
        panelsTelemetry.update(telemetry);
    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        shooter.update();
        statePathUpdate();

        // Telemetry
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
        panelsTelemetry.debug("Flywheel RPM", String.format("%.0f", shooter.getFlywheelRPM()));
        panelsTelemetry.debug("Target RPM", String.format("%.0f", shooter.getTargetVelocity()));

        panelsTelemetry.debug("=== LIMELIGHT ===");
        panelsTelemetry.debug("Valid Target", shooter.hasValidTarget() ? "YES" : "NO");
        panelsTelemetry.debug("Distance (in)", String.format("%.1f", shooter.getDistance()));
        panelsTelemetry.debug("Horizontal Offset", String.format("%.2f°", shooter.getHorizontalOffset()));
        panelsTelemetry.debug("Launch Angle", String.format("%.2f", shooter.getTargetAngle()));
        panelsTelemetry.update(telemetry);
    }
}