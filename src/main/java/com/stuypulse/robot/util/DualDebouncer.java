package com.stuypulse.robot.util;

import edu.wpi.first.wpilibj.Timer;

public class DualDebouncer {
  private final double riseTime;
  private final double fallTime;
  private final Timer timer = new Timer();
  private boolean baseline = false;

  public DualDebouncer(double riseSeconds, double fallSeconds) {
    riseTime = riseSeconds;
    fallTime = fallSeconds;
    timer.start();
  }

  public boolean calculate(boolean input) {
    if (input == baseline) {
      // no change from current output, reset the timer
      timer.reset();
    } else {
      double requiredTime = input ? riseTime : fallTime;
      if (timer.hasElapsed(requiredTime)) {
        baseline = input;
        timer.reset();
      }
    }
    return baseline;
  }
}
