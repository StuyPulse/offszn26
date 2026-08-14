// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package com.stuypulse.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public interface VisionConstants {
  public interface VisionSettings {
    final int RESET_IMU_INDEX = 1;

    // AprilTag layout
    final AprilTagFieldLayout APRIL_TAG_LAYOUT =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    // Basic filtering thresholds
    final double MAX_AMBIGUITY = 0.3;
    final double MAX_Z_ERROR = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    final double LINEAR_STD_DEV_BASELINE = 0.02; // Meters
    final double ANGULAR_STD_DEV_BASELINE = 0.06; // Radians

    // Multipliers to apply for MegaTag 2 observations
    final double LINEAR_STD_DEV_MEGATAG_2_FACTOR = 0.5; // More stable than full 3D solve
    final double ANGULAR_STD_DEV_MEGATAG_2_FACTOR =
        Double.POSITIVE_INFINITY; // No rotation data available

    final double BUZZ_DEBOUNCE = 0.25;
  }

  /**
   * Change: Remove hardcoded camera names (e.g. camera0Name, camera1Name, camera2Name)
   *
   * <p>Now uses a record to define every camera with its name, camera offset to the robot, and
   * standard deviation factor.
   *
   * <p>This allows for easier management of cameras and also being the ONE source of truth.
   *
   * <p>!!!! Note that the index of the Camera enum and the index of the camera data array must
   * match!!!!
   */
  enum Camera {
    RIGHT,
    LEFT,
    BACK
  }

  record CameraData(String name, Transform3d robotToCamera, double stdDevFactor) {}

  public interface CamerasList {
    final CameraData[] CAMERAS = {
      new CameraData(
          "limelight-right",
          new Transform3d(
              Units.inchesToMeters(-9.149),
              Units.inchesToMeters(15.080),
              Units.inchesToMeters(8.088),
              new Rotation3d(
                  Units.degreesToRadians(180),
                  Units.degreesToRadians(28.0),
                  Units.degreesToRadians(-80.203885))),
          1.0),
      new CameraData(
          "limelight-left",
          new Transform3d(
              Units.inchesToMeters(-2.490),
              Units.inchesToMeters(-14.8620),
              Units.inchesToMeters(5.676),
              new Rotation3d(
                  Units.degreesToRadians(0),
                  Units.degreesToRadians(14.955812),
                  Units.degreesToRadians(71.5))),
          1.0),
      new CameraData(
          "limelight-back",
          new Transform3d(
              Units.inchesToMeters(-10.676),
              Units.inchesToMeters(-12.969),
              Units.inchesToMeters(8.753),
              new Rotation3d(
                  Units.degreesToRadians(0),
                  Units.degreesToRadians(27.875),
                  Units.degreesToRadians(185.155825))),
          1.0)
    };
  }
}
