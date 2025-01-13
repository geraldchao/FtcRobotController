package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.SequenceSegment;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "drive")
public class TurnDebug extends LinearOpMode {
    public static double ANGLE = 360; // deg

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        // drive.turn(Math.toRadians(ANGLE));

        double targetHeading =  Math.toRadians(ANGLE * 1.07);

        TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(new Pose2d());
        // builder.setTurnConstraint(Math.toRadians(360), Math.toRadians(45));
        TrajectorySequence traj = builder
                .turn(targetHeading)
                .build();

        drive.followTrajectorySequence(traj);

        drive.followTrajectorySequenceAsync(traj);

        double lastAccel = 0;
        while (!Thread.currentThread().isInterrupted() && drive.isBusy()) {
            drive.updatePoseEstimate();
            Log.d("drive", Misc.formatInvariant("heading: %.2f tgt %.2f", Math.toDegrees(drive.getPoseEstimate().getHeading()), Math.toDegrees(targetHeading)));
            DriveSignal signal = drive.trajectorySequenceRunner.update(drive.getPoseEstimate(), drive.getPoseVelocity());
            if (signal != null) {
                Pose2d newVPose = signal.getVel();
                Pose2d newAPose = signal.getAccel();
                double newVelocity = newVPose.getHeading() > 0 ? newVPose.getHeading() : 0.0;
                double newAccel = newAPose.getHeading() != 0.0 ? lastAccel + (newAPose.getHeading()-lastAccel) / 2: 0.0;
                DriveSignal signalSmoothed = new DriveSignal(
                        new Pose2d(newVPose.getX(), newVPose.getY(), newVelocity),
                        new Pose2d(newAPose.getX(), newAPose.getY(), newAccel)
                );
                Log.d("drive", Misc.formatInvariant("signal: %s", signal.toString()));
                Log.d("drive", Misc.formatInvariant("signalSmoothed: %s", signalSmoothed.toString()));
                drive.setDriveSignal(signalSmoothed);
                lastAccel = newAccel;
            }
        }

    }

}
