package com.stuypulse.robot.constants;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;

public interface Field {
  public static final Distance TRENCH_HOOD_TOLERANCE = Inches.of(20);

  public static Distance WIDTH = Inches.of(317.00);
  public static Distance LENGTH = Inches.of(651.200);

  public static final Distance OPPONENT_ZONE_X = LENGTH.minus(Inches.of(158.6));

  public static final Pose2d HUB_CENTER =
      new Pose2d(Inches.of(182.11), WIDTH.div(2), new Rotation2d());
  public static final Pose2d HUB_FAR_RIGHT_CORNER =
      new Pose2d(Inches.of(205.6), WIDTH.div(2).plus(Inches.of(47 / 2.0)), Rotation2d.kZero);
  public static final Pose2d HUB_FAR_LEFT_CORNER =
      new Pose2d(Inches.of(205.6), WIDTH.div(2).plus(Inches.of(47 / 2.0)), Rotation2d.kZero);

  public static final Distance HUB_RADIUS = Inches.of(41.7 / 2.0);

  public static final Pose2d KB_POSE =
      new Pose2d(
          HUB_CENTER.getMeasureX().minus(HUB_RADIUS).minus(Inches.of(23.5)),
          Field.WIDTH.div(2).minus(Inches.of(6.5)),
          Rotation2d.kZero);

  public static final Distance OPPONENT_HUB_DS_X =
      LENGTH.minus(HUB_FAR_LEFT_CORNER.getMeasureX()).plus(HUB_RADIUS.times(2));

  public static final Pose2d INNER_LEFT_FERRY_ZONE =
      new Pose2d(Inches.of(31.5), WIDTH.minus(Inches.of(82.5)), new Rotation2d());

  public static final Pose2d INNER_RIGHT_FERRY_ZONE =
      new Pose2d(Inches.of(20.75), Inches.of(76).plus(Inches.of(48)), new Rotation2d());

  public static final Pose2d OUTER_LEFT_FERRY_ZONE =
      new Pose2d(Inches.of(31.5), WIDTH.minus(Inches.of(34.5)), new Rotation2d());

  public static final Pose2d OUTER_RIGHT_FERRY_ZONE =
      new Pose2d(Inches.of(20.75), Inches.of(76), new Rotation2d());

  public static final Distance FERRY_SWITCH_TRIGGER_METERS_FROM_EDGE = Inches.of(75);

  public final Pose2d TOWER_FAR_CENTER =
      new Pose2d(Inches.of(42.0), Inches.of(147.47), new Rotation2d());
  public final Pose2d TOWER_FAR_RIGHT =
      new Pose2d(Inches.of(42.0), Inches.of(147.47 - 23.5 - 10), new Rotation2d());
  public final Pose2d TOWER_FAR_LEFT =
      new Pose2d(Inches.of(42.0), Inches.of(147.47 + 23.5 - 5 + 3.5), new Rotation2d());
  public final Distance TOWER_BAR_DISPLACEMENT = Inches.of(11.38);

  public static final Distance BEHIND_HUB_TOLERANCE_X =
      Inches.of(144); // To extend the triangle vertex
  public static final Distance BEHIND_HUB_TOLERANCE_Y =
      Inches.of(12 + 2); // To extend base of triangle (colinear with back hub)

  public static final Pose2d BEHIND_HUB_TRIANGLE_VERTEX =
      new Pose2d(Inches.of(182.11).plus(BEHIND_HUB_TOLERANCE_X), WIDTH.div(2.0), new Rotation2d());

  public interface AllianceLeftTrench {
    public static final Pose2d leftEdge = new Pose2d(Inches.of(182.11), WIDTH, new Rotation2d());
    public static final Pose2d rightEdge =
        new Pose2d(Inches.of(182.11), WIDTH.minus(Inches.of(50.59)), new Rotation2d());
  }

  public interface AllianceRightTrench {
    public static final Pose2d leftEdge =
        new Pose2d(Inches.of(182.11), Inches.of(50.59), new Rotation2d());
    public static final Pose2d rightEdge =
        new Pose2d(Inches.of(182.11), Inches.of(0), new Rotation2d());
  }

  // OPPONENT SIDE, BUT LEFT/RIGHT RELATIVE TO YOUR ALLIANCE POV
  public interface OpponentLeftTrench {
    public static final Pose2d leftEdge =
        new Pose2d(LENGTH.minus(Inches.of(182.11)), WIDTH, new Rotation2d());
    public static final Pose2d rightEdge =
        new Pose2d(
            LENGTH.minus(Inches.of(182.11)), WIDTH.minus(Inches.of(50.59)), new Rotation2d());
  }

  // OPPONENT SIDE, BUT LEFT/RIGHT RELATIVE TO YOUR ALLIANCE POV
  public interface OpponentRightTrench {
    public static final Pose2d leftEdge =
        new Pose2d(LENGTH.minus(Inches.of(182.11)), Inches.of(50.59), new Rotation2d());
    public static final Pose2d rightEdge =
        new Pose2d(LENGTH.minus(Inches.of(182.11)), Inches.of(0), new Rotation2d());
  }

  public static Pose2d getFerryZonePose(Translation2d robot) {
    Distance fieldMidY = WIDTH.div(2);

    if (robot.getMeasureY().gt(fieldMidY)) {
      if (robot.getMeasureY().gt(WIDTH.minus(FERRY_SWITCH_TRIGGER_METERS_FROM_EDGE))) {
        return INNER_LEFT_FERRY_ZONE;
      } else {
        return OUTER_LEFT_FERRY_ZONE;
      }
    } else {
      if (robot.getMeasureY().lt(FERRY_SWITCH_TRIGGER_METERS_FROM_EDGE)) {
        return INNER_RIGHT_FERRY_ZONE;
      } else {
        return OUTER_RIGHT_FERRY_ZONE;
      }
    }
  }

  public static Pose2d transformToOppositeAlliance(Pose2d pose) {
    Pose2d rotated = pose.rotateBy(Rotation2d.fromDegrees(180));
    return new Pose2d(
        rotated.getTranslation().plus(new Translation2d(LENGTH, WIDTH)), rotated.getRotation());
  }

  public static final double[] ALL_TAGS = {
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26,
    27, 28, 29, 30, 31, 32
  };
}
