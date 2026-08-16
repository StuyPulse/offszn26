package com.stuypulse.robot.subsystems.intake;

import com.stuypulse.robot.constants.GlobalSettings;
import com.stuypulse.robot.subsystems.intake.IntakeConstants.*;
import com.stuypulse.robot.subsystems.intake.IntakeIO.IntakeIOOutputs;
import com.stuypulse.robot.subsystems.intake.IntakeIO.PivotIOOutputMode;
import com.stuypulse.robot.subsystems.intake.IntakeIO.RollerIOOutputMode;
import com.stuypulse.robot.util.FullSubsystem;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends FullSubsystem {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs;
  private final IntakeIOOutputs outputs;

  @AutoLogOutput(key = "States/Intake/Pivot")
  private PivotState pivotState;

  @AutoLogOutput(key = "States/Intake/Rollers")
  private RollerState rollerState;

  public Intake(IntakeIO io) {
    this.io = io;
    inputs = new IntakeIOInputsAutoLogged();
    outputs = new IntakeIOOutputs();

    setPivotState(PivotState.STOW);
    setRollerState(RollerState.STOP);
  }

  public enum PivotState {
    DEPLOY,
    STOW
  }

  public enum RollerState {
    INTAKE,
    OUTTAKE,
    STOP
  }

  private void setPivotState(PivotState state) {
    this.pivotState = state;
  }

  private void setRollerState(RollerState state) {
    this.rollerState = state;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    if (!GlobalSettings.EnabledSubsystems.INTAKE.get()) {
      stopPivotMotor();
      stopRollerMotors();

      return;
    }

    switch (pivotState) {
      case DEPLOY -> runPivotPosition(IntakeSettings.PIVOT_DEPLOY_ANGLE);

      case STOW -> runPivotPosition(IntakeSettings.PIVOT_STOW_ANGLE);
    }

    if (!canRunRollers()) {
      stopRollerMotors();

      return;
    }

    switch (rollerState) {
      case INTAKE -> runRollersDutyCycle(IntakeSettings.INTAKE_DUTY_CYCLE);

      case OUTTAKE -> runRollersDutyCycle(IntakeSettings.OUTTAKE_DUTY_CYCLE);

      case STOP -> stopRollerMotors();
    }
  }

  @Override
  public void periodicAfterScheduler() {
    io.applyOutputs(outputs);
  }

  private boolean canRunRollers() {
    return inputs.pivotMotorPosition.lte(IntakeSettings.ROLLER_START_THRESHOLD)
        && pivotState == PivotState.DEPLOY;
  }

  private void runPivotPosition(Angle position) {
    outputs.pivotMode = PivotIOOutputMode.POSITION;
    outputs.pivotTargetPosition = position;
  }

  private void runRollersDutyCycle(double dutyCycle) {
    outputs.rollerMode = RollerIOOutputMode.DUTY_CYCLE;
    outputs.rollerTargetDutyCycle = dutyCycle;
  }

  private void stopPivotMotor() {
    outputs.pivotMode = PivotIOOutputMode.STOP;
  }

  private void stopRollerMotors() {
    outputs.rollerMode = RollerIOOutputMode.STOP;
  }

  public Command intake() {
    return runOnce(
            () -> {
              setPivotState(PivotState.DEPLOY);
              setRollerState(RollerState.INTAKE);
            })
        .withName("Intake Intake");
  }

  public Command outtake() {
    return runOnce(
            () -> {
              setPivotState(PivotState.DEPLOY);
              setRollerState(RollerState.OUTTAKE);
            })
        .withName("Intake Outtake");
  }

  public Command stow() {
    return runOnce(
            () -> {
              setPivotState(PivotState.STOW);
              setRollerState(RollerState.STOP);
            })
        .withName("Intake Stow");
  }
}
