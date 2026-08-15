package com.stuypulse.robot.subsystems.feeder;

import com.stuypulse.robot.constants.GlobalSettings;
import com.stuypulse.robot.subsystems.feeder.FeederConstants.FeederSettings;
import com.stuypulse.robot.subsystems.feeder.FeederIO.FeederIOOutputMode;
import com.stuypulse.robot.subsystems.feeder.FeederIO.FeederIOOutputs;
import com.stuypulse.robot.util.FullSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Feeder extends FullSubsystem {
  private final FeederIO io;
  private final FeederIOInputsAutoLogged inputs;
  private final FeederIOOutputs outputs;

  @AutoLogOutput(key = "States/Feeder")
  private FeederState state;

  public Feeder(FeederIO io) {
    this.io = io;
    inputs = new FeederIOInputsAutoLogged();
    outputs = new FeederIOOutputs();

    setState(FeederState.STOP);
  }

  public enum FeederState {
    FORWARD,
    REVERSE,
    STOP
  }

  private void setState(FeederState state) {
    this.state = state;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Feeder", inputs);

    if (!GlobalSettings.EnabledSubsystems.FEEDER.get()) {
      stopMotor();

      return;
    }

    switch (state) {
      case FORWARD -> runDutyCycle(FeederSettings.FORWARD_DUTY_CYCLE);

      case REVERSE -> runDutyCycle(FeederSettings.REVERSE_DUTY_CYCLE);

      case STOP -> stopMotor();
    }
  }

  @Override
  public void periodicAfterScheduler() {
    io.applyOutputs(outputs);
  }

  private void runDutyCycle(double dutyCycle) {
    outputs.feederMode = FeederIOOutputMode.DUTY_CYCLE;
    outputs.targetDutyCycle = dutyCycle;
  }

  private void stopMotor() {
    outputs.feederMode = FeederIOOutputMode.STOP;
  }

  public Command runForward() {
    return runOnce(() -> setState(FeederState.FORWARD));
  }

  public Command runReverse() {
    return runOnce(() -> setState(FeederState.REVERSE));
  }

  public Command stop() {
    return runOnce(() -> setState(FeederState.STOP));
  }
}
