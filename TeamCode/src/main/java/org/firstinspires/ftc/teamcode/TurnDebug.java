package org.firstinspires.ftc.teamcode.drive.opmode;

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
    public static double ANGLE = 180; // deg

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        // drive.turn(Math.toRadians(ANGLE));


        TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(new Pose2d());
        // builder.setTurnConstraint(Math.toRadians(360), Math.toRadians(45));
        TrajectorySequence traj = builder
                .turn(Math.toRadians(ANGLE))
                .build();

        for (int i = 0; i < traj.size(); i++) {
            SequenceSegment segment = traj.get(i);
            Log.d("segment", Misc.formatInvariant("segment %d: %s", i, segment.toString()));
        }

        drive.followTrajectorySequenceAsync(traj);
        while (!Thread.currentThread().isInterrupted() && drive.isBusy()) {
            drive.updatePoseEstimate();
            Log.d("drive", Misc.formatInvariant("heading: %.2f", Math.toDegrees(drive.getPoseEstimate().getHeading())));
            DriveSignal signal = drive.trajectorySequenceRunner.update(drive.getPoseEstimate(), drive.getPoseVelocity());
            if (signal != null) {
                Log.d("drive", Misc.formatInvariant("signal: %s", signal.toString()));
                drive.setDriveSignal(signal);
            }

        }

    }

}
