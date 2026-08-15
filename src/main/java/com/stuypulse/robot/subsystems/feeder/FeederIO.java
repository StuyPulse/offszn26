package com.stuypulse.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

public interface FeederIO {
  @AutoLog
  public static class FeederIOInputs {
    public Current feederLeaderMotorSupplyCurrent = Amps.zero();
    public Current feederLeaderMotorStatorCurrent = Amps.zero();
    public Temperature feederLeaderMotorTemperature = Celsius.zero();
    public Angle feederLeaderMotorPosition = Degrees.zero();
    public Voltage feederLeaderMotorAppliedVoltage = Volts.zero();
    public AngularVelocity feederLeaderMotorVelocity = RPM.zero();

    public Current feederFollowerMotorSupplyCurrent = Amps.zero();
    public Current feederFollowerMotorStatorCurrent = Amps.zero();
    public Temperature feederFollowerMotorTemperature = Celsius.zero();
    public Angle feederFollowerMotorPosition = Degrees.zero();
    public Voltage feederFollowerMotorAppliedVoltage = Volts.zero();
    public AngularVelocity feederFollowerMotorVelocity = RPM.zero();
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
