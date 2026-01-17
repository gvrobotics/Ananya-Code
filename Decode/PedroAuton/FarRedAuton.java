package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;

@Autonomous
public class FarRedAuton extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer, intakeTimer;

    private boolean firstRowPathStarted = false;
    private boolean secondRowPathStarted = false;



    // ---------- FLYWHEEL SETUP ----------
    private Flywheel shooter = new Flywheel();
    private boolean shotsTriggered = false;

    public enum PathState {
        // START POSITION_END POSITION
        // DRIVE -> MOVEMENT STATE
        // SHOOT -> ATTEMPT TO SCORE ARTIFACT
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD,
        DRIVE_SHOOT_ENDPOS;
    }


    PathState pathState;

    private final Pose startPose = new Pose(83.85441941074525, 9.233968804159433, Math.toRadians(90));
    private final Pose shootPose = new Pose(92.63570190641249, 99.3275563258232, Math.toRadians(45));
    private final Pose endPose = new Pose(101.2512998266898, 71.06759098786829, Math.toRadians(0));
    private PathChain driveStartPosShootPos, driveShootPosEndPose;

    public void buildPaths() {
        // Put in coordinates for starting pose -> ending pose
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(),shootPose.getHeading())
                .build();

        driveShootPosEndPose = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
                .build();

    }

    public void statePathUpdate(){
        switch(pathState) {
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos, true);
                shooter.fly1.setVelocity(1300);
                shooter.fly2.setVelocity(1300);
                setPathState(PathState.SHOOT_PRELOAD); //reset timer & make new state
                break;

            case SHOOT_PRELOAD:
                // Check if follower done it's path
                if (!follower.isBusy()) {
                    // requested shots yet
                    if (!shotsTriggered) {
                        shooter.fireShots(3);
                        shotsTriggered = true;
                    }
                    else if (shotsTriggered && !shooter.isBusy()) {
                        // shots are done free to transition
                        follower.followPath(driveShootPosEndPose, true);
                        setPathState(PathState.DRIVE_SHOOT_ENDPOS);
                    }
                }
                break;

            case DRIVE_SHOOT_ENDPOS:
                // All done!
                if(!follower.isBusy()) {
                    telemetry.addLine("Done all paths");
                }

            default:
                telemetry.addLine("No State Commanded");
                break;
        }

    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
        firstRowPathStarted = false;
        secondRowPathStarted = false;
        shotsTriggered = false;
    }

    @Override
    public void init() {
        pathState = pathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        intakeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        // TODO add in any other init mechanisms
        shooter.init(hardwareMap);

        buildPaths();
        follower.setPose(startPose);
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

        telemetry.addData("Path state", pathState.toString());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());

        telemetry.addLine("---- SHOOTER ----");
        telemetry.addData("Shooter State", shooter.getState());
        telemetry.addData("Shots Remaining", shooter.getShotsRemaining());
        telemetry.addData("Flywheel RPM", shooter.getFlywheelRPM());

    }
}
