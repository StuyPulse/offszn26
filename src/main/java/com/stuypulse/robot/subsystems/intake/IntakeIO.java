package com.stuypulse.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public Current pivotMotorSupplyCurrent = Amps.zero();
    public Current pivotMotorStatorCurrent = Amps.zero();
    public Temperature pivotMotorTemperature = Celsius.zero();
    public Angle pivotMotorPosition = Degrees.zero();
    public Voltage pivotMotorAppliedVoltage = Volts.zero();
    public AngularVelocity pivotMotorVelocity = DegreesPerSecond.zero();

    public Current rollerLeaderMotorSupplyCurrent = Amps.zero();
    public Current rollerLeaderMotorStatorCurrent = Amps.zero();
    public Temperature rollerLeaderMotorTemperature = Celsius.zero();
    public Angle rollerLeaderMotorPosition = Degrees.zero();
    public Voltage rollerLeaderMotorAppliedVoltage = Volts.zero();
    public AngularVelocity rollerLeaderMotorVelocity = DegreesPerSecond.zero();

    public Current rollerFollowerMotorSupplyCurrent = Amps.zero();
    public Current rollerFollowerMotorStatorCurrent = Amps.zero();
    public Temperature rollerFollowerMotorTemperature = Celsius.zero();
    public Angle rollerFollowerMotorPosition = Degrees.zero();
    public Voltage rollerFollowerMotorAppliedVoltage = Volts.zero();
    public AngularVelocity rollerFollowerMotorVelocity = DegreesPerSecond.zero();
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public static enum PivotIOOutputMode {
    POSITION,
    TORQUE_CURRENT,
    STOP
  }

  public static enum RollerIOOutputMode {
    DUTY_CYCLE,
    STOP
  }

  public static class IntakeIOOutputs {
    @AutoLogOutput(key = "Intake/Pivot/Output Mode")
    public PivotIOOutputMode pivotMode = PivotIOOutputMode.POSITION;

    @AutoLogOutput(key = "Intake/Pivot/Gain Slot")
    public int pivotGainSlot = 0;

    @AutoLogOutput(key = "Intake/Pivot/Target Position")
    public Angle pivotTargetPosition = Degrees.zero();

    @AutoLogOutput(key = "Intake/Pivot/Target Torque Current")
    public Current pivotTargetTorqueCurrent = Amps.zero();

    @AutoLogOutput(key = "Intake/Rollers/Output Mode")
    public RollerIOOutputMode rollerMode = RollerIOOutputMode.DUTY_CYCLE;

    @AutoLogOutput(key = "Intake/Rollers/Target Duty Cycle")
    public double rollerTargetDutyCycle = 0.0;
  }

  public default void applyOutputs(IntakeIOOutputs outputs) {}
}
