

package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;

import java.util.List;

@Autonomous(name="TestAuto", group="Linear Opmode")
public class TestAuto extends LinearOpMode {
    public static double ROBOT_WIDTH_INCHES = 16.2;
    public static double ROBOT_LENGTH_INCHES = 12;

    private DcMotorEx armM;
    private Servo leftClawS;
    private Servo rightClawS;
    private Servo wristS;
    private Servo armS;

    // private FirstVisionProcessor visionProcessor;
    private VisionPortal visionPortal;
    private ColorBlobLocatorProcessor colorLocator;

    @Override
    public void runOpMode() throws InterruptedException {
        /*
        initCamera(ColorRange.BLUE);
        while (opModeIsActive() || opModeInInit()) {
            telemetry.addData("preview on/off", "... Camera Stream\n");
            getCameraBlobs();
        }
        */

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

    private void initCamera(ColorRange targetColorRange) {
        colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(targetColorRange)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 0.8, 1.0, -0.8))  // search central 1/4 of camera view
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(5)                               // Smooth the transitions between different colors in image
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        // telemetry.setMsTransmissionInterval(50);   // Speed up telemetry updates, Just use for debugging.
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
    }

    @SuppressLint("DefaultLocale")
    public void getCameraBlobs() {

        // Read the current list
        List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();

        ColorBlobLocatorProcessor.Util.filterByArea(1000, 20000, blobs);  // filter out very small blobs.
        ColorBlobLocatorProcessor.Util.sortByArea(SortOrder.DESCENDING, blobs);

        telemetry.addLine(" Area Density Aspect  Center");

        // Display the size (area) and center location for each Blob.
        for(ColorBlobLocatorProcessor.Blob b : blobs)
        {
            RotatedRect boxFit = b.getBoxFit();
            telemetry.addLine(String.format("%5d  %4.2f   %5.2f  (%3d,%3d)",
                    b.getContourArea(), b.getDensity(), b.getAspectRatio(), (int) boxFit.center.x, (int) boxFit.center.y));

            // raw initial numbers
            // grabber location:
            // area 3600, density 0.9, aspect ratio 1.7, center x y (185,147)

            // sample ~3 inches in front:
            // area 3600, density 0.9, AR 2., center (160, 118)

            // sample ~3 inches in front, 2 inches right
            // area 3340, density 0.95, AR 3, center (248, 104)

            // sample ~3 inches in front, 2 inches left
            // area 4000, density 0.9, AR 1.44, center (91, 122)
        }

        telemetry.update();
        sleep(500);
    }
}

