/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.constants;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

public interface DriverConstants {

    public interface Driver {
        int DRIVER_INDEX = 0;
        double BUZZ_TIME = 1.0;
        double BUZZ_INTENSITY = 1.0;

        public interface Drive {
            double DEADBAND = 0.05;
            double RC = 0.05;
            int POWER = 2;
        }

        public interface Turn {
            double DEADBAND = 0.05;
            double RC = 0.05;
            int POWER = 2;
        }
    }

    public interface DriveConstraints {
        LinearVelocity MAX_VELOCITY = MetersPerSecond.of(1.16);
        LinearVelocity MAX_VELOCITY_SOTM = MetersPerSecond.of(1.75);
        LinearVelocity MAX_VELOCITY_FOTM = MetersPerSecond.of(4.16);

        AngularVelocity MAX_ANGULAR_VEL = DegreesPerSecond.of(300.0);
        AngularVelocity MAX_ANGULAR_VEL_SOTM = DegreesPerSecond.of(75.0);
        AngularVelocity MAX_ANGULAR_VEL_FOTM = DegreesPerSecond.of(150.0);

        LinearAcceleration MAX_ACCEL = MetersPerSecondPerSecond.of(15.0);
        LinearAcceleration MAX_ACCEL_SOTM = MetersPerSecondPerSecond.of(4.0);
        LinearAcceleration MAX_ACCEL_FOTM = MetersPerSecondPerSecond.of(15.0);
        AngularAcceleration MAX_ANGULAR_ACCEL = DegreesPerSecondPerSecond.of(900.0);

        PathConstraints PATH_CONSTRAINTS =
                new PathConstraints(MAX_VELOCITY, MAX_ACCEL, MAX_ANGULAR_VEL, MAX_ANGULAR_ACCEL);
    }
}
