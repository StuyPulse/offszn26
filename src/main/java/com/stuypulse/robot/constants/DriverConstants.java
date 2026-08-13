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
    public final LinearVelocity MAX_VELOCITY = MetersPerSecond.of(4.16);
    public final LinearVelocity MAX_VELOCITY_SOTM = MetersPerSecond.of(1.75);
    public final LinearVelocity MAX_VELOCITY_FOTM = MetersPerSecond.of(4.16);

    public final AngularVelocity MAX_ANGULAR_VEL = DegreesPerSecond.of(300.0);
    public final AngularVelocity MAX_ANGULAR_VEL_SOTM = DegreesPerSecond.of(75.0);
    public final AngularVelocity MAX_ANGULAR_VEL_FOTM = DegreesPerSecond.of(150.0);

    public final LinearAcceleration MAX_ACCEL = MetersPerSecondPerSecond.of(15.0);
    public final LinearAcceleration MAX_ACCEL_SOTM = MetersPerSecondPerSecond.of(4.0);
    public final LinearAcceleration MAX_ACCEL_FOTM = MetersPerSecondPerSecond.of(15.0);
    public final AngularAcceleration MAX_ANGULAR_ACCEL_RAD_PER_S_SQUARED =
        DegreesPerSecondPerSecond.of(900.0);
  }
}
