package com.stuypulse.robot.util.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class AlignmentUtil {
  public static Rotation2d getTargetAlignmentAngle(Pose2d currentPose, Pose2d targetPose) {
    double atan =
        Math.atan2(targetPose.getY() - currentPose.getY(), targetPose.getX() - currentPose.getX());

    // The shooter shoots out of the back of the robot
    return new Rotation2d(atan - Math.PI);
  }
}
