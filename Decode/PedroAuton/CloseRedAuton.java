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
        //------FIRST ROW------
        DRIVE_SHOOTPOS_FIRSTROW_START,
        DRIVE_FIRSTROW_START_TO_END,
        DRIVE_FIRST_ENDPOS_SHOOTPOS,
        SHOOT_FIRSTROW,
        //-----SECOND ROW-------
        DRIVE_SHOOT_TO_SECONDROW_START,
        DRIVE_SECONDROW_START_TO_END,
        DRIVE_SECOND_ENDPOS_SHOOTPOS,
        SHOOT_SECONDROW,
        DRIVE_SHOOT_ENDPOS;
    }


    PathState pathState;

    private final Pose startPose = new Pose(117.54592720970538, 129.2755632582322, Math.toRadians(45));
    private final Pose shootPose = new Pose(92.63570190641249, 99.3275563258232, Math.toRadians(45));
    private final Pose secondRowShootPose = new Pose(83.9116117850953, 135.63778162911612, Math.toRadians(0));
    private final Pose firstRowStart = new Pose(100.69844020797227, 83.6065857885615, Math.toRadians(0));
    //TODO Change Y Values
    private final Pose firstRowEnd = new Pose(129.00346620450608, 83.6065857885615, Math.toRadians(0));
    private final Pose secondRowStart = new Pose(100.69844020797227, 59.82668977469672, Math.toRadians(0));
    private final Pose secondRowEnd = new Pose(129.00346620450608, 59.82668977469672, Math.toRadians(0));
    private final Pose endPose = new Pose(101.2512998266898, 71.06759098786829, Math.toRadians(0));
    private PathChain driveStartPosShootPos, driveShootPosFirstRowStartPose, driveFirstRowStartToEndPose, driveFirstRowtoShootPose, driveShootPosSecondRowStartPose, driveSecondRowStartToEndPose, driveSecondRowtoShootPose, driveShootPosEndPose;

    public void buildPaths() {
        // Put in coordinates for starting pose -> ending pose
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(),shootPose.getHeading())
                .build();
        //---------------------FIRST ROW START----------------------
        driveShootPosFirstRowStartPose = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, firstRowStart))
                .setLinearHeadingInterpolation(shootPose.getHeading(),firstRowStart.getHeading())
                .build();
        driveFirstRowStartToEndPose = follower.pathBuilder()
                .addPath(new BezierLine(firstRowStart, firstRowEnd))
                .setLinearHeadingInterpolation(firstRowStart.getHeading(),firstRowEnd.getHeading())
                .build();
        driveFirstRowtoShootPose = follower.pathBuilder()
                .addPath(new BezierLine(firstRowEnd, shootPose))
                .setLinearHeadingInterpolation(firstRowEnd.getHeading(), shootPose.getHeading())
                .build();
        //---------------------SECOND ROW START------------------------
        driveShootPosSecondRowStartPose = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, secondRowStart))
                .setLinearHeadingInterpolation(shootPose.getHeading(),firstRowStart.getHeading())
                .build();
        driveSecondRowStartToEndPose = follower.pathBuilder()
                .addPath(new BezierLine(secondRowStart, secondRowEnd))
                .setLinearHeadingInterpolation(secondRowStart.getHeading(), secondRowEnd.getHeading())
                .build();
        driveSecondRowtoShootPose = follower.pathBuilder()
                .addPath(new BezierLine(secondRowEnd, secondRowShootPose))
                .setLinearHeadingInterpolation(secondRowEnd.getHeading(), secondRowShootPose.getHeading())
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
                        follower.followPath(driveShootPosFirstRowStartPose, true);
                        setPathState(PathState.DRIVE_SHOOTPOS_FIRSTROW_START);
                    }
                }
                break;

//-------------------------------FIRST ROW--------------------------------------
            case DRIVE_SHOOTPOS_FIRSTROW_START:
                follower.followPath(driveShootPosFirstRowStartPose, true);
                shooter.intake1.setPower(0.7);
                shooter.intake2.setPower(0.7);
                setPathState(PathState.DRIVE_FIRSTROW_START_TO_END);
                intakeTimer.resetTimer();
                break;

            case DRIVE_FIRSTROW_START_TO_END:
                if (!firstRowPathStarted) {
                    follower.followPath(driveFirstRowStartToEndPose, true);
                    firstRowPathStarted = true;
                }

                if (!follower.isBusy()) {
                    shooter.intake1.setPower(0);
                    shooter.intake2.setPower(0);
                    firstRowPathStarted = false;
                    setPathState(PathState.DRIVE_FIRST_ENDPOS_SHOOTPOS);
                }
                break;

            case DRIVE_FIRST_ENDPOS_SHOOTPOS:
                follower.followPath(driveFirstRowtoShootPose, true);
                shooter.fly1.setVelocity(1300);
                shooter.fly2.setVelocity(1300);
                setPathState(PathState.SHOOT_FIRSTROW);
                break;

            case SHOOT_FIRSTROW:
                // Check if follower done it's path
                if (!follower.isBusy()) {
                    // requested shots yet
                    if (!shotsTriggered) {
                        shooter.fireShots(3);
                        shotsTriggered = true;
                    }
                    else if (shotsTriggered && !shooter.isBusy()) {
                        // shots are done free to transition
                        follower.followPath(driveShootPosSecondRowStartPose, true);
                        setPathState(PathState.DRIVE_SHOOT_TO_SECONDROW_START);
                    }
                }
                break;
//-------------------------------SECOND ROW--------------------------------------
            case DRIVE_SHOOT_TO_SECONDROW_START:
                follower.followPath(driveShootPosSecondRowStartPose, true);
                shooter.intake1.setPower(0.7);
                shooter.intake2.setPower(0.7);
                setPathState(PathState.DRIVE_SECONDROW_START_TO_END);
                intakeTimer.resetTimer();
                break;

            case DRIVE_SECONDROW_START_TO_END:
                if (!secondRowPathStarted) {
                    follower.followPath(driveSecondRowStartToEndPose, true);
                    secondRowPathStarted = true;
                }

                if (!follower.isBusy()) {
                    shooter.intake1.setPower(0);
                    shooter.intake2.setPower(0);
                    secondRowPathStarted = false;
                    setPathState(PathState.DRIVE_SECOND_ENDPOS_SHOOTPOS);
                }
                break;

            case DRIVE_SECOND_ENDPOS_SHOOTPOS:
                follower.followPath(driveSecondRowtoShootPose, true);
                shooter.fly1.setVelocity(1300);
                shooter.fly2.setVelocity(1300);
                setPathState(PathState.SHOOT_SECONDROW);
                break;

            case SHOOT_SECONDROW:
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
