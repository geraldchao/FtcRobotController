package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    ^
 *    |
 *    | ( x direction)
 *    |
 *    v
 *    <----( y direction )---->

 *        (forward)
 *    /--------------\
 *    |     ____     |
 *    |     ----     |    <- Perpendicular Wheel
 *    |           || |
 *    |           || |    <- Parallel Wheel
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
public class TwoWheelTrackingLocalizer extends TwoTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 2000;
    public static double WHEEL_RADIUS = 0.944882; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double PARALLEL_X = -1; // X is the up and down direction
    public static double PARALLEL_Y = 8.2; // Y is the strafe direction

    public static double PERPENDICULAR_X = 0;
    public static double PERPENDICULAR_Y = 6;

    public static double X_MULTIPLIER = 1; // Multiplier in the X direction
    public static double Y_MULTIPLIER = 1; // Multiplier in the Y direction


    // Parallel/Perpendicular to the forward axis
    // Parallel wheel is parallel to the forward axis
    // Perpendicular is perpendicular to the forward axis
    private Encoder parallelEncoder, perpendicularEncoder;

    private SampleMecanumDrive drive;

    private List<Integer> lastEncPositions, lastEncVels;

    public TwoWheelTrackingLocalizer(HardwareMap hardwareMap, SampleMecanumDrive drive, List<Integer> lastTrackingEncPositions, List<Integer> lastTrackingEncVels) {

        super(Arrays.asList(
                new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
                new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
        ));

        this.drive = drive;
        lastEncPositions = lastTrackingEncPositions;
        lastEncVels = lastTrackingEncVels;

        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "parallelEncoder"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "perpendicularEncoder"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        perpendicularEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public double getHeading() {
        return drive.getRawExternalHeading();
    }

    @Override
    public Double getHeadingVelocity() {
        return drive.getExternalHeadingVelocity();
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {

        int parallelPos = (int)((double) parallelEncoder.getCurrentPosition() * X_MULTIPLIER);
        int perpendicularPos = (int)((double)perpendicularEncoder.getCurrentPosition() * Y_MULTIPLIER);

        lastEncPositions.clear();
        lastEncPositions.add(parallelPos);
        lastEncPositions.add(perpendicularPos);

        return Arrays.asList(
                encoderTicksToInches(parallelPos),
                encoderTicksToInches(perpendicularPos)
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        int parallelVel = (int) (parallelEncoder.getCorrectedVelocity() * X_MULTIPLIER);
        int perpendicularVel = (int) (perpendicularEncoder.getCorrectedVelocity() * Y_MULTIPLIER);

        lastEncVels.clear();
        lastEncVels.add(parallelVel);
        lastEncVels.add(perpendicularVel);

        return Arrays.asList(
                encoderTicksToInches(parallelVel),
                encoderTicksToInches(perpendicularVel)
        );

    }
}