package com.stuypulse.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.constants.GlobalSettings;
import com.stuypulse.robot.subsystems.intake.IntakeConstants.*;
import edu.wpi.first.units.measure.*;

public class IntakeIOReal implements IntakeIO {
  private final TalonFX pivotMotor;
  private final TalonFX rollerLeaderMotor;
  private final TalonFX rollerFollowerMotor;

  private final DutyCycleOut rollerLeaderController;
  private final Follower rollerFollowerController;
  private final PositionVoltage pivotPositionController;

  private final StatusSignal<Angle> pivotPosition;
  private final StatusSignal<Current> pivotSupplyCurrent;
  private final StatusSignal<Current> pivotStatorCurrent;
  private final StatusSignal<Temperature> pivotTemperature;
  private final StatusSignal<Voltage> pivotAppliedVoltage;
  private final StatusSignal<AngularVelocity> pivotVelocity;

  private final StatusSignal<Angle> rollerLeaderPosition;
  private final StatusSignal<Current> rollerLeaderSupplyCurrent;
  private final StatusSignal<Current> rollerLeaderStatorCurrent;
  private final StatusSignal<Temperature> rollerLeaderTemperature;
  private final StatusSignal<Voltage> rollerLeaderAppliedVoltage;
  private final StatusSignal<AngularVelocity> rollerLeaderVelocity;

  private final StatusSignal<Angle> rollerFollowerPosition;
  private final StatusSignal<Current> rollerFollowerSupplyCurrent;
  private final StatusSignal<Current> rollerFollowerStatorCurrent;
  private final StatusSignal<Temperature> rollerFollowerTemperature;
  private final StatusSignal<Voltage> rollerFollowerAppliedVoltage;
  private final StatusSignal<AngularVelocity> rollerFollowerVelocity;

  public IntakeIOReal() {
    this.pivotMotor = new TalonFX(IntakeDeviceIds.PIVOT, GlobalSettings.RIO);
    this.rollerLeaderMotor = new TalonFX(IntakeDeviceIds.ROLLER_LEADER, GlobalSettings.RIO);
    this.rollerFollowerMotor = new TalonFX(IntakeDeviceIds.ROLLER_FOLLOWER, GlobalSettings.RIO);

    IntakeMotorConfigs.PIVOT_CONFIG.configure(pivotMotor);
    IntakeMotorConfigs.ROLLER_CONFIG.configure(rollerLeaderMotor);
    IntakeMotorConfigs.ROLLER_CONFIG.configure(rollerFollowerMotor);

    this.rollerLeaderController = new DutyCycleOut(0).withEnableFOC(true);
    this.rollerFollowerController =
        new Follower(rollerLeaderMotor.getDeviceID(), MotorAlignmentValue.Opposed);
    this.pivotPositionController = new PositionVoltage(0).withEnableFOC(true);

    rollerFollowerMotor.setControl(rollerFollowerController);
    pivotMotor.setControl(pivotPositionController);

    this.pivotPosition = pivotMotor.getPosition();
    this.pivotSupplyCurrent = pivotMotor.getSupplyCurrent();
    this.pivotStatorCurrent = pivotMotor.getStatorCurrent();
    this.pivotTemperature = pivotMotor.getDeviceTemp();
    this.pivotAppliedVoltage = pivotMotor.getMotorVoltage();
    this.pivotVelocity = pivotMotor.getVelocity();

    this.rollerLeaderPosition = rollerLeaderMotor.getPosition();
    this.rollerLeaderSupplyCurrent = rollerLeaderMotor.getSupplyCurrent();
    this.rollerLeaderStatorCurrent = rollerLeaderMotor.getStatorCurrent();
    this.rollerLeaderTemperature = rollerLeaderMotor.getDeviceTemp();
    this.rollerLeaderAppliedVoltage = rollerLeaderMotor.getMotorVoltage();
    this.rollerLeaderVelocity = rollerLeaderMotor.getVelocity();

    this.rollerFollowerPosition = rollerFollowerMotor.getPosition();
    this.rollerFollowerSupplyCurrent = rollerFollowerMotor.getSupplyCurrent();
    this.rollerFollowerStatorCurrent = rollerFollowerMotor.getStatorCurrent();
    this.rollerFollowerTemperature = rollerFollowerMotor.getDeviceTemp();
    this.rollerFollowerAppliedVoltage = rollerFollowerMotor.getMotorVoltage();
    this.rollerFollowerVelocity = rollerFollowerMotor.getVelocity();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        pivotPosition,
        pivotSupplyCurrent,
        pivotStatorCurrent,
        pivotTemperature,
        pivotAppliedVoltage,
        pivotVelocity,
        rollerLeaderPosition,
        rollerLeaderSupplyCurrent,
        rollerLeaderStatorCurrent,
        rollerLeaderTemperature,
        rollerLeaderAppliedVoltage,
        rollerLeaderVelocity,
        rollerFollowerPosition,
        rollerFollowerSupplyCurrent,
        rollerFollowerStatorCurrent,
        rollerFollowerTemperature,
        rollerFollowerAppliedVoltage,
        rollerFollowerVelocity);

    inputs.pivotMotorPosition = pivotPosition.getValue();
    inputs.pivotMotorSupplyCurrent = pivotSupplyCurrent.getValue();
    inputs.pivotMotorStatorCurrent = pivotStatorCurrent.getValue();
    inputs.pivotMotorTemperature = pivotTemperature.getValue();
    inputs.pivotMotorAppliedVoltage = pivotAppliedVoltage.getValue();
    inputs.pivotMotorVelocity = pivotVelocity.getValue();

    inputs.rollerLeaderMotorPosition = rollerLeaderPosition.getValue();
    inputs.rollerLeaderMotorSupplyCurrent = rollerLeaderSupplyCurrent.getValue();
    inputs.rollerLeaderMotorStatorCurrent = rollerLeaderStatorCurrent.getValue();
    inputs.rollerLeaderMotorTemperature = rollerLeaderTemperature.getValue();
    inputs.rollerLeaderMotorAppliedVoltage = rollerLeaderAppliedVoltage.getValue();
    inputs.rollerLeaderMotorVelocity = rollerLeaderVelocity.getValue();

    inputs.rollerFollowerMotorPosition = rollerFollowerPosition.getValue();
    inputs.rollerFollowerMotorSupplyCurrent = rollerFollowerSupplyCurrent.getValue();
    inputs.rollerFollowerMotorStatorCurrent = rollerFollowerStatorCurrent.getValue();
    inputs.rollerFollowerMotorTemperature = rollerFollowerTemperature.getValue();
    inputs.rollerFollowerMotorAppliedVoltage = rollerFollowerAppliedVoltage.getValue();
    inputs.rollerFollowerMotorVelocity = rollerFollowerVelocity.getValue();
  }

  @Override
  public void applyOutputs(IntakeIOOutputs outputs) {
    switch (outputs.pivotMode) {
      case POSITION -> pivotMotor.setControl(
          pivotPositionController.withPosition(outputs.pivotTargetPosition));
      case STOP -> pivotMotor.stopMotor();
    }

    switch (outputs.rollerMode) {
      case DUTY_CYCLE -> rollerLeaderMotor.setControl(
          rollerLeaderController.withOutput(outputs.rollerTargetDutyCycle));
      case STOP -> {
        rollerLeaderMotor.stopMotor();
        rollerFollowerMotor.stopMotor();
        rollerFollowerMotor.setControl(rollerFollowerController);
      }
    }
  }
}
