package com.stuypulse.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    public Current hoodMotorSupplyCurrent = Amps.zero();
    public Current hoodMotorStatorCurrent = Amps.zero();
    public Temperature hoodMotorTemperature = Celsius.zero();
    public Angle hoodMotorPosition = Degrees.zero();
    public Voltage hoodMotorAppliedVoltage = Volts.zero();
    public AngularVelocity hoodMotorVelocity = RPM.zero();
  }

  public default void updateInputs(HoodIOInputs inputs) {}

  public static enum HoodIOOutputMode {
    POSITION,
    STOP
  }

  public static class HoodIOOutputs {
    @AutoLogOutput(key = "Hood/Output Mode")
    public HoodIOOutputMode hoodMode = HoodIOOutputMode.POSITION;

    @AutoLogOutput(key = "Hood/Target Position")
    public Angle hoodTargetPosition = Degrees.zero();
  }

  public default void applyOutputs(HoodIOOutputs outputs) {}
}
