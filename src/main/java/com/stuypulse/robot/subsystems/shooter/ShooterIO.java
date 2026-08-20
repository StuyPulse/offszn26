package com.stuypulse.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.stuypulse.robot.util.logged.LoggedTalonFX.TalonFXInputs;
import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public TalonFXInputs topLeftMotorInputs = new TalonFXInputs();
    public TalonFXInputs bottomLeftMotorInputs = new TalonFXInputs();

    public TalonFXInputs topRightMotorInputs = new TalonFXInputs();
    public TalonFXInputs bottomRightMotorInputs = new TalonFXInputs();
  }

  public enum ShooterIOOutputMode {
    STOP,
    VELOCITY_TORQUE_CURRENT_FOC
  }

  public static class ShooterIOOutputs {
    @AutoLogOutput(key = "Shooter/TargetVelocity")
    public AngularVelocity targetVelocity = RPM.zero();

    @AutoLogOutput(key = "Shooter/Output Mode")
    public ShooterIOOutputMode mode = ShooterIOOutputMode.VELOCITY_TORQUE_CURRENT_FOC;
  }

  default void updateInputs(ShooterIOInputs inputs) {}
  ;

  default void applyOutputs(ShooterIOOutputs outputs) {}
  ;
}
