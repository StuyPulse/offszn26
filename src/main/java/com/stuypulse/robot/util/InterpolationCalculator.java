package com.stuypulse.robot.util;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.RPM;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.InterpolationConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.Optional;

public class InterpolationCalculator {
    private static final InterpolatingDoubleTreeMap shooterRPMInterpolation;
    private static final InterpolatingDoubleTreeMap hoodPositionInterpolation;
    private static final InterpolatingDoubleTreeMap shooterFerryRPMInterpolation;

    private static Optional<InterpolatedShotInfo> cachedInterpolatedShotInfo;
    private static Optional<AngularVelocity> cachedInterpolatedFerryRPM;

    static {
        shooterRPMInterpolation = new InterpolatingDoubleTreeMap();
        hoodPositionInterpolation = new InterpolatingDoubleTreeMap();
        shooterFerryRPMInterpolation = new InterpolatingDoubleTreeMap();

        for (double[] pair : InterpolationConstants.DISTANCE_RPM_INTERPOLATION_VALUES) {
            shooterRPMInterpolation.put(pair[0], pair[1]);
        }

        for (double[] pair : InterpolationConstants.DISTANCE_ANGLE_INTERPOLATION_VALUES) {
            hoodPositionInterpolation.put(pair[0], pair[1]);
        }

        for (double[] pair : InterpolationConstants.FERRY_DISTANCE_RPM_INTERPOLATION) {
            shooterFerryRPMInterpolation.put(pair[0], pair[1]);
        }
    }

    private InterpolationCalculator() {}

    public static void clearMemoized() {
        cachedInterpolatedShotInfo = Optional.empty();
        cachedInterpolatedFerryRPM = Optional.empty();
    }

    public static AngularVelocity getInterpolatedShotRPM(Pose2d currentPose) {
        if (cachedInterpolatedShotInfo.isEmpty()) {
            cachedInterpolatedShotInfo = Optional.of(interpolateShotInfo(currentPose));
        }

        return cachedInterpolatedShotInfo.get().shooterVelocity();
    }

    public static Angle getInterpolatedShotHoodPosition(Pose2d currentPose) {
        if (cachedInterpolatedShotInfo.isEmpty()) {
            cachedInterpolatedShotInfo = Optional.of(interpolateShotInfo(currentPose));
        }

        return cachedInterpolatedShotInfo.get().hoodAngle();
    }

    public static AngularVelocity getInterpolatedFerryRPM(Pose2d currentPose) {
        if (cachedInterpolatedFerryRPM.isEmpty()) {
            cachedInterpolatedFerryRPM = Optional.of(interpolateFerryingInfo(currentPose));
        }

        return cachedInterpolatedFerryRPM.get();
    }

    public record InterpolatedShotInfo(Angle hoodAngle, AngularVelocity shooterVelocity) {}

    public static InterpolatedShotInfo interpolateShotInfo(Pose2d currentPose) {
        return interpolateShotInfo(currentPose, Field.HUB_CENTER);
    }

    public static InterpolatedShotInfo interpolateShotInfo(Pose2d currentPose, Pose2d targetPose) {
        Translation2d hubTranslation = targetPose.getTranslation();
        Translation2d currentTranslation = currentPose.getTranslation();

        double distanceMeters = currentTranslation.getDistance(hubTranslation);

        Angle targetAngle = Radians.of(hoodPositionInterpolation.get(distanceMeters));
        AngularVelocity targetRPM = RPM.of(shooterRPMInterpolation.get(distanceMeters));

        return new InterpolatedShotInfo(targetAngle, targetRPM);
    }

    public static AngularVelocity interpolateFerryingInfo(Pose2d currentPose) {
        Pose2d ferryPose = Field.getFerryZonePose(currentPose.getTranslation());

        return interpolateFerryingInfo(currentPose, ferryPose);
    }

    public static AngularVelocity interpolateFerryingInfo(Pose2d currentPose, Pose2d targetPose) {
        Translation2d currentTranslation = currentPose.getTranslation();
        Translation2d ferryPose = targetPose.getTranslation();

        double distanceMeters = currentTranslation.getDistance(ferryPose);

        AngularVelocity targetRPM = RPM.of(shooterFerryRPMInterpolation.get(distanceMeters));

        return targetRPM;
    }
}
