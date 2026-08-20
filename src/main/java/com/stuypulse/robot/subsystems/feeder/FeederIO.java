package com.stuypulse.robot.subsystems.feeder;

import com.stuypulse.robot.util.logged.LoggedTalonFX.TalonFXInputs;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

public interface FeederIO {
  @AutoLog
  public static class FeederIOInputs {
    public TalonFXInputs feederLeaderInputs = new TalonFXInputs();
    public TalonFXInputs feederFollowerInputs = new TalonFXInputs();
  }

  public default void updateInputs(FeederIOInputs inputs) {}

  public static enum FeederIOOutputMode {
    DUTY_CYCLE,
    STOP
  }

  public static class FeederIOOutputs {
    @AutoLogOutput(key = "Feeder/Output Mode")
    public FeederIOOutputMode feederMode = FeederIOOutputMode.DUTY_CYCLE;

    @AutoLogOutput(key = "Feeder/Target Duty Cycle")
    public double targetDutyCycle = 0;
  }

  public default void applyOutputs(FeederIOOutputs outputs) {}
}
