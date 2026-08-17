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

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public interface VisionConstants {
    public interface VisionSettings {
        int RESET_IMU_INDEX = 1;

        // Basic filtering thresholds
        double MAX_AMBIGUITY = 0.3;
        double MAX_Z_ERROR = 0.75;

        // Standard deviation baselines, for 1 meter distance and 1 tag
        // (Adjusted automatically based on distance and # of tags)
        double LINEAR_STD_DEV_BASELINE = 0.02; // Meters
        double ANGULAR_STD_DEV_BASELINE = 0.06; // Radians

        // Multipliers to apply for MegaTag 2 observations
        double LINEAR_STD_DEV_MEGATAG_2_FACTOR = 0.5; // More stable than full 3D solve
        double ANGULAR_STD_DEV_MEGATAG_2_FACTOR =
                Double.POSITIVE_INFINITY; // No rotation data available

        double BUZZ_DEBOUNCE = 0.25;

        double FUEL_RADIUS = Units.inchesToMeters(2.955);
    }

    /**
     * !!!! Note that the index of the Camera enum and the index of the camera data array must
     * match!!!!
     */
    enum Camera {
    // Add names here for each camera, in the order they are defined in the
    // CamerasList.CAMERAS array
    }

    record CameraData(String name, Transform3d robotToCamera, double stdDevFactor) {}

    public interface CamerasList {
        final CameraData[] CAMERAS = {};
    }
}
