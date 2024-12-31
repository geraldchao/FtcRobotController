package com.example.meepmeepexample;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;



public class MeepMeepExample {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        // https://learnroadrunner.com/trajectory-sequence.html#overview

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(16, 14)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(15, -62, Math.toRadians(90)))
                                .lineTo(new Vector2d(10, -30))
                                // hang specimen
                                .addDisplacementMarker(() -> {
                                })
                                .lineToLinearHeading(new Pose2d(28, -40, Math.PI*3/2))
                                .lineTo(new Vector2d(42, -12))
                                .strafeLeft(6)
                                .lineTo(new Vector2d(48, -55))
                                .lineTo(new Vector2d(48, -12))
                                .strafeLeft(10)
                                .lineTo(new Vector2d(58, -55))
                                .lineTo(new Vector2d(58, -12))
                                .strafeLeft(6)
                                // .splineTo(new Vector2d(60, -60), Math.toRadians(-90))
                                .lineTo(new Vector2d(60, -55))
                                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}