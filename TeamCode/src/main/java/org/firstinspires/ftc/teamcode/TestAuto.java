

package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;


@Autonomous(name="TestAuto", group="Linear Opmode")
public class TestAuto extends LinearOpMode {
    public static double ROBOT_WIDTH_INCHES = 16.2;
    public static double ROBOT_LENGTH_INCHES = 12;


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        double START_POSITION_X = -1 - (ROBOT_WIDTH_INCHES / 2);
        // start position of red side, left of center
        Pose2d startPose = new Pose2d(START_POSITION_X, -6*12 + ROBOT_LENGTH_INCHES / 2, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

            /*
        //auto code goes here
        reAdjust();
        hangSpecimen(19);
        pickUpSample(42);
        placeSample();

        drive(0.2, -10, -10, "straight");
        drive(0.0, 0, 0, "straight");
        moveArm(0.2, 5, "lowerArm");
        sleep(500);
        drive(0.2, 55, 55, "turnRight");
        drive(0.2, 39, 39, "left");
        drive(0.2, 12, 12, "straight");
         */
        Pose2d latestPose = startPose;
        latestPose = hangSpecimen(46 - ROBOT_LENGTH_INCHES / 2, 8, drive, latestPose);
        latestPose = pickUpSample(48 - (-START_POSITION_X) + ROBOT_WIDTH_INCHES / 2, 4, drive, latestPose); // fwdDist should be -1?
        latestPose = placeSample(18, 8, drive, latestPose);
        gotoAscentZone( 20, 39, 12, drive, latestPose);
    }

    public Pose2d gotoAscentZone(double backDist, double leftDist, double fwdDist, SampleMecanumDrive drive, Pose2d startPose) {
        TrajectorySequence backTraj = drive.trajectorySequenceBuilder(startPose)
                .back(backDist)
                .build();
        TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(backTraj.end());
        // builder.setTurnConstraint(Math.toRadians(360), Math.toRadians(45));
        TrajectorySequence leftTraj = builder
                .turn(Math.toRadians(15) - backTraj.end().getHeading())
                .splineToLinearHeading(new Pose2d(-16 - ROBOT_LENGTH_INCHES/ 2, -12), Math.toRadians(0))
                /*
                .turn(Math.toRadians(0) - backTraj.end().getHeading())
                .strafeLeft(leftDist)
                .forward(fwdDist)
                 */
                .build();

        /*
            drive(0.2, -10, -10, "straight");
            drive(0.0, 0, 0, "straight");
            */
        drive.followTrajectorySequence(backTraj);

        /*
        moveArm(0.2, 5, "lowerArm");
        */
        sleep(500);
        /*
        drive(0.2, 55, 55, "turnRight");
        drive(0.2, 39, 39, "left");
        drive(0.2, 12, 12, "straight");
         */
        drive.followTrajectorySequence(leftTraj);
        return leftTraj.end();

    }

    public Pose2d hangSpecimen(double fwdDist, double backDist, SampleMecanumDrive drive, Pose2d startPose){
        /*
        moveArm(0.1, 8, "raiseArm");
        wristMid();
        sleep(2150);
        drive(0.3, distance, distance, "straight");
        drive(0.1, 1, 1, "straight");
        drive(0.0, 0, 0, "straight");
        moveArm(0.4, 4, "lowerArm");
        sleep(1500);
        drive(0.3, -4, -4, "straight");
        drive(0.0, 0, 0, "straight");

         */

        TrajectorySequence forwardTraj = drive.trajectorySequenceBuilder(startPose)
                // .forward(fwdDist)
                .lineTo(new Vector2d(startPose.getX(), startPose.getY() + fwdDist))
                .build();
        TrajectorySequence backTraj = drive.trajectorySequenceBuilder(forwardTraj.end())
                .back(backDist)
                .build();

        /*
        moveArm(0.1, 8, "raiseArm");
        wristMid();
         */
        sleep(2150);
        drive.followTrajectorySequence(forwardTraj);
        /*
        moveArm(0.4, 4, "lowerArm");
         */
        sleep(1500);
        drive.followTrajectorySequence(backTraj);
        return backTraj.end();

    }

    public Pose2d pickUpSample(double leftDist, double fwdDist, SampleMecanumDrive drive, Pose2d startPose){
        TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(startPose);
        // builder.setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(6));
        TrajectorySequence leftTraj = builder
                .strafeLeft(leftDist)
                .forward(6) // fudge factor
                // .lineToConstantHeading(new Vector2d(-48-2, -42))
                .build();

        TrajectorySequence fwdTraj = drive.trajectorySequenceBuilder(leftTraj.end())
                .lineToConstantHeading(new Vector2d(-48-2, -28))
                // .forward(fwdDist)
                .build();
        /*
        openClaw();
        wristUp();
        moveArm(0.3, 9, "lowerArm");
        */
        sleep(1000);
        /*
        drive(0.3, left, left, "left");
        drive(0.1, -1, -1, "straight");
        drive(0.0, 0, 0, "straight");
        */
        drive.followTrajectorySequence(leftTraj);
        drive.followTrajectorySequence(fwdTraj);
        /*
        wristDown();
        */
        sleep(500);
        /*
        closeClaw();
         */
        sleep(1000);
        return fwdTraj.end();
    }

    public Pose2d placeSample(double backDist, double fwdDist, SampleMecanumDrive drive, Pose2d startPose){
        /*
        armDown();
        wristUp();
         */
        sleep(500);
        /*
        drive(0.3, -7, -7, "straight");
        drive(0.0, 0, 0, "straight");
        drive(0.3, 30, 30, "turnLeft");
         */
        TrajectorySequence toBasketTraj = drive.trajectorySequenceBuilder(startPose)
                // .back(backDist)
                .lineToConstantHeading(new Vector2d(startPose.getX(), -48))
                .turn(Math.toRadians(180+45 + 3) - startPose.getHeading() )
                .build();
        drive.followTrajectorySequence(toBasketTraj);
        /*
        moveArm(0.4, 10, "raiseArm");
        */
        sleep(2500);
      /*drive(0.3, 5, 5, "straight");
      drive(0.0, 0, 0, "straight");*/
        TrajectorySequence fwdTraj = drive.trajectorySequenceBuilder(toBasketTraj.end())
                // .forward(fwdDist)
                .lineToConstantHeading(new Vector2d(-60, -60))
                .build();
        drive.followTrajectorySequence(fwdTraj);
        /*
        armMid();
        wristDown();
        openClaw();
         */
        sleep(1000);
        return fwdTraj.end();
    }
/*
    private void drive(double speed, int leftTarget, int rightTarget, String direction) {

        telemetry.addData("Drive", direction);
        telemetry.update();
    }
*/
}

