package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;

@Autonomous
public class CloseRedAuton extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    // ---------- FLYWHEEL SETUP ----------
    private Flywheel shooter = new Flywheel();
    private boolean shotsTriggered = false;

    public enum PathState {
        // START POSITION_END POSITION
        // DRIVE -> MOVEMENT STATE
        // SHOOT -> ATTEMPT TO SCORE ARTIFACT
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD,
        DRIVE_SHOOTPOS_FIRSTROW_START,
        DRIVE_FIRSTROW_START_TO_END,
        DRIVE_SHOOT_ENDPOS
    }


    PathState pathState;

    private final Pose startPose = new Pose(110.05892547660312, 135.51473136915078, Math.toRadians(90));
    private final Pose shootPose = new Pose(83.35528596187177, 90.84228769497398, Math.toRadians(45));
    private final Pose firstRowStart = new Pose(101.94627383015599, 83.6065857885615, Math.toRadians(0));
    private final Pose firstRowEnd = new Pose(137.98786828422877, 83.6065857885615, Math.toRadians(0));
    private final Pose endPose = new Pose(71.74870017331021, 103.22876949740034, Math.toRadians(180)); //add ending pose here
    private PathChain driveStartPosShootPos, driveShootPosFirstRowStartPose, driveFirstRowStartToEndPose, driveShootPosEndPose;

    public void buildPaths() {
        // Put in coordinates for starting pose -> ending pose
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(),shootPose.getHeading())
                .build();
        driveShootPosFirstRowStartPose = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, firstRowStart))
                .setLinearHeadingInterpolation(shootPose.getHeading(),firstRowStart.getHeading())
                .build();
        driveFirstRowStartToEndPose = follower.pathBuilder()
                .addPath(new BezierLine(firstRowStart, firstRowEnd))
                .setLinearHeadingInterpolation(firstRowStart.getHeading(),firstRowEnd.getHeading())
                .build();
        driveShootPosEndPose = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .setLinearHeadingInterpolation(firstRowEnd.getHeading(), endPose.getHeading())
                .build();
    }

    public void statePathUpdate(){
        switch(pathState) {
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD); //reset timer & make new state
                break;

            case DRIVE_SHOOTPOS_FIRSTROW_START:
                follower.followPath(driveShootPosFirstRowStartPose, true);
                shooter.intake1.setVelocity(1500);
                shooter.intake2.setVelocity(1500);
                setPathState(PathState.DRIVE_FIRSTROW_START_TO_END);
                break;

            case DRIVE_FIRSTROW_START_TO_END:
                follower.followPath(driveFirstRowStartToEndPose, true);

                // KEEP intake on while driving
                shooter.intake1.setVelocity(1500);
                shooter.intake2.setVelocity(1500);

                // ONLY transition once path is finished
                if (!follower.isBusy()) {
                    shooter.intake1.setVelocity(0);
                    shooter.intake2.setVelocity(0);
                    setPathState(PathState.DRIVE_SHOOT_ENDPOS);
                }
                break;

            case SHOOT_PRELOAD:
                // Check if follower done it's path?
                if (!follower.isBusy()) {
                    // requested shots yet
                    if (!shotsTriggered) {
                        shooter.fireShots(3);
                        shotsTriggered = true;
                    }
                    else if (shotsTriggered && !shooter.isBusy()) {
                        // shots are done free to transition
                        follower.followPath(driveShootPosFirstRowStartPose, true);
                        setPathState(PathState.DRIVE_SHOOTPOS_FIRSTROW_START);
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

        shotsTriggered = false;
    }

    @Override
    public void init() {
        pathState = pathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
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
