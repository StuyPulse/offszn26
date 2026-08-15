package com.stuypulse.robot.constants;

public interface InterpolationConstants {
  // TODO: Placeholder Values

  double[][] DISTANCE_ANGLE_INTERPOLATION_VALUES = {
    {0.96, Math.toRadians(15.0)},
    {1.22, Math.toRadians(20.0)},
    {2.15, Math.toRadians(27.0)},
    {3.38, Math.toRadians(34.0)},
    {4.43, Math.toRadians(39.0)},
    {5.66, Math.toRadians(39.0)},
    {6.44, Math.toRadians(44.0)}
  };

  double[][] DISTANCE_RPM_INTERPOLATION_VALUES = {
    {0.96, 2800.0},
    {1.22, 2600.0},
    {2.15, 2805.0},
    {3.38, 3075.0},
    {4.43, 3350.0},
    {5.66, 3650.0},
    {6.44, 3800.0},
    {8.23, 4500.0} // THIS POINT IS AN EXTRAPOLATION
  };

  double[][] FERRY_DISTANCE_RPM_INTERPOLATION = {
    {1.0, 2000.0},
    {5.16, 3300.0},
    {6.94, 3600.0},
    {7.87, 3800.0},
    {9.77, 4300.0}, // TODO: ADD DATA BACK IN COMP
    {10.694, 4700.0}, // STARTING FROM HERE THE DATA IS EXTRAPOLATED
    {11.516, 4900.0}
  };
}
