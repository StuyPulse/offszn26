package com.stuypulse.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Degrees;

import com.stuypulse.robot.constants.GlobalSettings;
import com.stuypulse.robot.subsystems.hood.HoodConstants.*;
import com.stuypulse.robot.subsystems.hood.HoodIO.HoodIOOutputMode;
import com.stuypulse.robot.subsystems.hood.HoodIO.HoodIOOutputs;
import com.stuypulse.robot.util.FullSubsystem;
import com.stuypulse.robot.util.InterpolationCalculator;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Hood extends FullSubsystem {

  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs;
  private final HoodIOOutputs outputs;

  private final InterpolationCalculator interpolator;

  private boolean atTolerance;

  @AutoLogOutput(key = "States/Hood")
  private HoodState state;

  public Hood(HoodIO io, InterpolationCalculator interpolator) {
    this.io = io;
    inputs = new HoodIOInputsAutoLogged();
    outputs = new HoodIOOutputs();

    this.interpolator = interpolator;

    atTolerance = false;

    setState(HoodState.STOW);
  }

  public enum HoodState {
    SHOOT,
    FERRY,
    STOW
  }

  private void setState(HoodState state) {
    this.state = state;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    if (!GlobalSettings.EnabledSubsystems.HOOD.get()) {
      stopMotor();

      return;
    }

    switch (state) {
      case SHOOT -> runPosition(interpolator.getInterpolatedShotHoodPosition());

      case FERRY -> runPosition(HoodSettings.FERRY_ANGLE);

      case STOW -> runPosition(HoodSettings.STOW_ANGLE);
    }
  }

  @Override
  public void periodicAfterScheduler() {
    io.applyOutputs(outputs);
  }

  @AutoLogOutput(key = "Hood/At Tolerance")
  public boolean atTolerance() {
    return atTolerance;
  }

  private void runPosition(Angle position) {
    outputs.hoodMode = HoodIOOutputMode.POSITION;
    outputs.hoodTargetPosition = position;

    atTolerance =
        inputs.hoodInputs.position.minus(position).abs(Degrees)
            <= HoodSettings.TOLERANCE.in(Degrees);
  }

  private void stopMotor() {
    outputs.hoodMode = HoodIOOutputMode.STOP;
  }

  public Command shoot() {
    return runOnce(() -> setState(HoodState.SHOOT)).withName("Hood Shoot");
  }

  public Command ferry() {
    return runOnce(() -> setState(HoodState.FERRY)).withName("Hood Ferry");
  }

  public Command stow() {
    return runOnce(() -> setState(HoodState.STOW)).withName("Hood Stow");
  }
}
