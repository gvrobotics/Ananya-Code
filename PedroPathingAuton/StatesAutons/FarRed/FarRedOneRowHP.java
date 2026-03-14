package org.firstinspires.ftc.teamcode.PedroPathingAuton.StatesAutons.FarRed;

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

@Autonomous(group = "Far Red", name = "8. Far Red One Row HP", preselectTeleOp = "Tele 5440")
public class FarRedOneRowHP extends OpMode {

    private TelemetryManager panelsTelemetry;
    private Follower follower;
    private Flywheel shooter = new Flywheel();

    // Timers
    private Timer pathTimer, opModeTimer;

    // State flags
    private boolean pathStarted = false, shotsTriggered = false;

    private boolean USE_AUTO_ADJUST = true;

    public enum PathState {
        DRIVE_STARTPOS_SHOOT_POS,
        ALIGN_AND_SHOOT_PRELOAD,
        //------ROW------
        DRIVE_SHOOTPOS_ROW_START,
        DRIVE_ROW_START_TO_END,
        DRIVE_ROW_ENDPOS_SHOOTPOS,
        ALIGN_AND_SHOOT_ROW,

        //------HP 1------
        DRIVE_SHOOT_TO_HP,
        DRIVE_HP1_START_TO_END,
        DRIVE_HP1_END_TO_SHOOT,
        ALIGN_AND_SHOOTHP,

        //------HP 2------
        DRIVE_SHOOT_TO_HP2,
        DRIVE_HP2_START_TO_END,
        DRIVE_HP2_END_TO_SHOOT,
        ALIGN_AND_SHOOTHP2,
        DRIVE_SHOOT_ENDPOS,
        COMPLETE
    }

    PathState pathState;

    private double rowStart = 99, rowEnd = 132.5;
    private double thirdRowPos = 35.5;

    // ================= POSES =================
    private final Pose startPose = new Pose(84.6, 9, Math.toRadians(90));
    private final Pose thirdRowStart = new Pose(rowStart, thirdRowPos, Math.toRadians(0));
    private final Pose thirdRowEnd = new Pose(rowEnd, thirdRowPos, Math.toRadians(0));
    private final Pose shootPose = new Pose(86.0, 12.11, Math.toRadians(65));
    private final Pose hpStart = new Pose(133.8, 16.7, Math.toRadians(-19));
    private final Pose hpEnd = new Pose(132.08, 9.8, Math.toRadians(-27));
    private final Pose endPose = new Pose(108.5, 12.2, Math.toRadians(0));


    // ================= PATHS =================
    private PathChain driveStartPosShootPos,
            driveShootPosRowStartPose,
            driveRowStartToEndPose,
            driveRowtoShootPose,
            driveShootPosHp1Pose, driveHp1StartToEndPose, driveHp1ToShootPose,
            driveShootPosEndPose;

    public void buildPaths() {

        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        //---------------------ROW START----------------------
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

        driveShootPosHp1Pose = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, hpStart))
                .setLinearHeadingInterpolation(shootPose.getHeading(), hpStart.getHeading())
                .build();

        driveHp1StartToEndPose = follower.pathBuilder()
                .addPath(new BezierLine(hpStart, hpEnd))
                .setLinearHeadingInterpolation(hpStart.getHeading(), hpEnd.getHeading())
                .setTValueConstraint(0.4)
                .setTimeoutConstraint(500)
                .build();

        driveHp1ToShootPose = follower.pathBuilder()
                .addPath(new BezierLine(hpEnd, shootPose))
                .setLinearHeadingInterpolation(hpEnd.getHeading(), shootPose.getHeading())
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

            // ==================== ROW ====================
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
                    handleShoot(PathState.DRIVE_SHOOT_TO_HP);
                }
                break;

            // ==================== HP ====================
            case DRIVE_SHOOT_TO_HP:
                handleDriveToRowStart(driveShootPosHp1Pose, PathState.DRIVE_HP1_START_TO_END);
                break;

            case DRIVE_HP1_START_TO_END:
                handleRowCollection(driveHp1StartToEndPose, PathState.DRIVE_HP1_END_TO_SHOOT);
                break;

            case DRIVE_HP1_END_TO_SHOOT:
                follower.followPath(driveHp1ToShootPose, true);
                setPathState(PathState.ALIGN_AND_SHOOTHP);
                break;

            case ALIGN_AND_SHOOTHP:
                if (!follower.isBusy()) {
                    handleShoot(PathState.DRIVE_SHOOT_TO_HP2);
                }
                break;

            // ==================== HP2 ====================
            case DRIVE_SHOOT_TO_HP2:
                handleDriveToRowStart(driveShootPosHp1Pose, PathState.DRIVE_HP2_START_TO_END);
                break;

            case DRIVE_HP2_START_TO_END:
                handleRowCollection(driveHp1StartToEndPose, PathState.DRIVE_HP2_END_TO_SHOOT);
                break;

            case DRIVE_HP2_END_TO_SHOOT:
                follower.followPath(driveHp1ToShootPose, true);
                setPathState(PathState.ALIGN_AND_SHOOTHP2);
                break;

            case ALIGN_AND_SHOOTHP2:
                if (!follower.isBusy()) {
                    handleShoot(PathState.DRIVE_SHOOT_ENDPOS);
                }
                break;

            // ==================== END ====================

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
                shooter.stopFlywheel();
                shooter.stopIntake();
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
            shooter.intake1.setPower(0.7);
            shooter.intake2.setPower(0.7);
            pathStarted = true;
        }

        if (!follower.isBusy()) {
            shooter.intake1.setPower(0.1);
            shooter.intake2.setPower(0.1);
            setPathState(nextState);
        }
    }

    // ============== HELPER METHOD: Shoot ==============

    private void handleShoot(PathState nextState) {

        if (!shotsTriggered) {
            shooter.fireShots(3);
            shotsTriggered = true;
        } else if (!shooter.isBusy()) {
            setPathState(nextState);
        }
    }

    public void setPathState(PathState newState) {

        pathState = newState;
        pathTimer.resetTimer();

        pathStarted = false;
        shotsTriggered = false;
    }

    @Override
    public void init() {

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;

        pathTimer = new Timer();
        opModeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        shooter.init(hardwareMap);
        shooter.setAutoAdjust(USE_AUTO_ADJUST);

        buildPaths();
        follower.setPose(startPose);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.debug("Auto-Adjust", USE_AUTO_ADJUST ? "ENABLED" : "DISABLED");
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

        // Telemetry

        panelsTelemetry.debug("=== PATH ===");
        panelsTelemetry.debug("State", pathState.toString());
        panelsTelemetry.debug("Progress", follower.getDistanceRemaining());
        panelsTelemetry.debug("Intake", shooter.intake1.getPower());

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
        panelsTelemetry.debug("Launch Angle", String.format("%.2f", shooter.getTargetAngle()));

        panelsTelemetry.update(telemetry);
    }
}