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
import java.util.function.Supplier;

public class InterpolationCalculator {
  private final InterpolatingDoubleTreeMap shooterRPMInterpolation;
  private final InterpolatingDoubleTreeMap hoodPositionInterpolation;
  private final InterpolatingDoubleTreeMap shooterFerryRPMInterpolation;

  private final Supplier<Pose2d> poseSupplier;

  private Optional<InterpolatedShotInfo> cachedInterpolatedShotInfo;
  private Optional<AngularVelocity> cachedInterpolatedFerryRPM;

  public InterpolationCalculator(Supplier<Pose2d> poseSupplier) {
    this.poseSupplier = poseSupplier;

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

  public AngularVelocity getInterpolatedShotRPM() {
    if (cachedInterpolatedShotInfo.isEmpty()) {
      cachedInterpolatedShotInfo = Optional.of(interpolateShotInfo());
    }

    return cachedInterpolatedShotInfo.get().shooterVelocity();
  }

  public Angle getInterpolatedShotHoodPosition() {
    if (cachedInterpolatedShotInfo.isEmpty()) {
      cachedInterpolatedShotInfo = Optional.of(interpolateShotInfo());
    }

    return cachedInterpolatedShotInfo.get().hoodAngle();
  }

  public AngularVelocity getInterpolatedFerryRPM() {
    if (cachedInterpolatedFerryRPM.isEmpty()) {
      cachedInterpolatedFerryRPM = Optional.of(interpolateFerryingInfo());
    }

    return cachedInterpolatedFerryRPM.get();
  }

  public record InterpolatedShotInfo(Angle hoodAngle, AngularVelocity shooterVelocity) {}
  ;

  public InterpolatedShotInfo interpolateShotInfo() {
    return interpolateShotInfo(poseSupplier.get(), Field.HUB_CENTER);
  }

  public InterpolatedShotInfo interpolateShotInfo(Pose2d currentPose, Pose2d targetPose) {
    Translation2d hubTranslation = targetPose.getTranslation();
    Translation2d currentTranslation = currentPose.getTranslation();

    double distanceMeters = currentTranslation.getDistance(hubTranslation);

    Angle targetAngle = Radians.of(hoodPositionInterpolation.get(distanceMeters));
    AngularVelocity targetRPM = RPM.of(shooterRPMInterpolation.get(distanceMeters));

    return new InterpolatedShotInfo(targetAngle, targetRPM);
  }

  public AngularVelocity interpolateFerryingInfo() {
    Pose2d currentPose = poseSupplier.get();
    Pose2d ferryPose = Field.getFerryZonePose(currentPose.getTranslation());

    return interpolateFerryingInfo(currentPose, ferryPose);
  }

  public AngularVelocity interpolateFerryingInfo(Pose2d currentPose, Pose2d targetPose) {
    Translation2d currentTranslation = currentPose.getTranslation();
    Translation2d ferryPose = targetPose.getTranslation();

    double distanceMeters = currentTranslation.getDistance(ferryPose);

    AngularVelocity targetRPM = RPM.of(shooterFerryRPMInterpolation.get(distanceMeters));

    return targetRPM;
  }
}
