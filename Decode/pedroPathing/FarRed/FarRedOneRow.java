package org.firstinspires.ftc.teamcode.pedroPathing.FarRed;

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

@Autonomous(group = "Far Red", name = "2. Far Red One Row", preselectTeleOp = "Tele 5440")
public class FarRedOneRow extends OpMode {

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
        DRIVE_SHOOTPOS_ROW_START,
        DRIVE_ROW_START_TO_END,
        DRIVE_ROW_ENDPOS_SHOOTPOS,
        ALIGN_AND_SHOOT_ROW,
        DRIVE_SHOOT_ENDPOS,
        COMPLETE
    }

    private PathState pathState;
    private double rowStart = 99, rowEnd = 132.5;

    // ===== POSES =====
    private final Pose startPose = new Pose(84.6, 9, Math.toRadians(90));
    private final Pose shootPose = new Pose(82.5, 21.1, Math.toRadians(65));
    private final Pose thirdRowStart = new Pose(rowStart, 35.3, Math.toRadians(0));
    private final Pose thirdRowEnd = new Pose(rowEnd, 35.3, Math.toRadians(0));
    private final Pose endPose = new Pose(108.5, 12.2, Math.toRadians(0));

    // ===== PATHS =====
    private PathChain driveStartPosShootPos, driveShootPosRowStartPose,
            driveRowStartToEndPose, driveRowtoShootPose, driveShootPosEndPose;

    public void buildPaths() {
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        driveShootPosRowStartPose = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, thirdRowStart))
                .setLinearHeadingInterpolation(shootPose.getHeading(), thirdRowStart.getHeading())
                .build();

        driveRowStartToEndPose = follower.pathBuilder()
                .addPath(new BezierLine(thirdRowStart, thirdRowEnd))
                .setLinearHeadingInterpolation(thirdRowStart.getHeading(), thirdRowEnd.getHeading())
                .build();

        driveRowtoShootPose = follower.pathBuilder()
                .addPath(new BezierLine(thirdRowEnd, shootPose))
                .setLinearHeadingInterpolation(thirdRowEnd.getHeading(), shootPose.getHeading())
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
                    handleShoot(PathState.DRIVE_SHOOTPOS_ROW_START);
                }
                break;

            // ===== ROW COLLECTION =====

            case DRIVE_SHOOTPOS_ROW_START:
                handleDriveToRowStart(driveShootPosRowStartPose, PathState.DRIVE_ROW_START_TO_END);
                break;

            case DRIVE_ROW_START_TO_END:
                handleRowCollection(driveRowStartToEndPose, PathState.DRIVE_ROW_ENDPOS_SHOOTPOS);
                break;

            case DRIVE_ROW_ENDPOS_SHOOTPOS:
                follower.followPath(driveRowtoShootPose, true);
                setPathState(PathState.ALIGN_AND_SHOOT_ROW);
                break;

            case ALIGN_AND_SHOOT_ROW:
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