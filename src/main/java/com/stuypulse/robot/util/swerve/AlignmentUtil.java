package com.stuypulse.robot.util.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class AlignmentUtil {
  public static Rotation2d getTargetAlignmentAngle(Pose2d currentPose, Pose2d targetPose) {

    // The shooter shoots out of the back of the robot
    return new Rotation2d(
            targetPose.getX() - currentPose.getX(), targetPose.getY() - currentPose.getY())
        .plus(Rotation2d.k180deg);
  }
}
