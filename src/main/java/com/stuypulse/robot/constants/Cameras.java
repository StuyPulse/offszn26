/************************ PROJECT PHIL ************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/
package com.stuypulse.robot.constants;

import edu.wpi.first.math.geometry.Pose3d;

public interface Cameras {
  public static record Camera(String name, Pose3d location) {}
  ;
}
