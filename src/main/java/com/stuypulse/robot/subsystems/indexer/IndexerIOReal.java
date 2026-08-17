package com.stuypulse.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.constants.GlobalSettings;
import com.stuypulse.robot.subsystems.indexer.IndexerConstants.*;
import edu.wpi.first.units.measure.*;

public class IndexerIOReal implements IndexerIO {
  private final TalonFX indexerLeader;
  private final TalonFX indexerFollower;

  private final DutyCycleOut indexerLeaderController;
  private final Follower indexerFollowerController;

  private final StatusSignal<Current> indexerLeaderSupplyCurrent;
  private final StatusSignal<Current> indexerLeaderStatorCurrent;
  private final StatusSignal<Temperature> indexerLeaderTemperature;
  private final StatusSignal<Voltage> indexerLeaderAppliedVoltage;
  private final StatusSignal<AngularVelocity> indexerLeaderVelocity;

  private final StatusSignal<Current> indexerFollowerSupplyCurrent;
  private final StatusSignal<Current> indexerFollowerStatorCurrent;
  private final StatusSignal<Temperature> indexerFollowerTemperature;
  private final StatusSignal<Voltage> indexerFollowerAppliedVoltage;
  private final StatusSignal<AngularVelocity> indexerFollowerVelocity;

  public IndexerIOReal() {
    indexerLeader = new TalonFX(IndexerDeviceIds.LEADER, GlobalSettings.RIO);
    indexerFollower = new TalonFX(IndexerDeviceIds.FOLLOWER, GlobalSettings.RIO);

    IndexerMotorConfigs.INDEXER_CONFIG.configure(indexerLeader);
    IndexerMotorConfigs.INDEXER_CONFIG.configure(indexerFollower);

    indexerLeaderController = new DutyCycleOut(0).withEnableFOC(true);
    indexerFollowerController =
        new Follower(indexerLeader.getDeviceID(), MotorAlignmentValue.Opposed);

    indexerFollower.setControl(indexerFollowerController);

    indexerLeaderSupplyCurrent = indexerLeader.getSupplyCurrent();
    indexerLeaderStatorCurrent = indexerLeader.getStatorCurrent();
    indexerLeaderTemperature = indexerLeader.getDeviceTemp();
    indexerLeaderAppliedVoltage = indexerLeader.getMotorVoltage();
    indexerLeaderVelocity = indexerLeader.getVelocity();

    indexerFollowerSupplyCurrent = indexerFollower.getSupplyCurrent();
    indexerFollowerStatorCurrent = indexerFollower.getStatorCurrent();
    indexerFollowerTemperature = indexerFollower.getDeviceTemp();
    indexerFollowerAppliedVoltage = indexerFollower.getMotorVoltage();
    indexerFollowerVelocity = indexerFollower.getVelocity();
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        indexerLeaderSupplyCurrent,
        indexerLeaderStatorCurrent,
        indexerLeaderTemperature,
        indexerLeaderAppliedVoltage,
        indexerLeaderVelocity,
        indexerFollowerSupplyCurrent,
        indexerFollowerStatorCurrent,
        indexerFollowerTemperature,
        indexerFollowerAppliedVoltage,
        indexerFollowerVelocity);

    inputs.indexerLeaderMotorSupplyCurrent = indexerLeaderSupplyCurrent.getValue();
    inputs.indexerLeaderMotorStatorCurrent = indexerLeaderStatorCurrent.getValue();
    inputs.indexerLeaderMotorTemperature = indexerLeaderTemperature.getValue();
    inputs.indexerLeaderMotorAppliedVoltage = indexerLeaderAppliedVoltage.getValue();
    inputs.indexerLeaderMotorVelocity = indexerLeaderVelocity.getValue();

    inputs.indexerFollowerMotorSupplyCurrent = indexerFollowerSupplyCurrent.getValue();
    inputs.indexerFollowerMotorStatorCurrent = indexerFollowerStatorCurrent.getValue();
    inputs.indexerFollowerMotorTemperature = indexerFollowerTemperature.getValue();
    inputs.indexerFollowerMotorAppliedVoltage = indexerFollowerAppliedVoltage.getValue();
    inputs.indexerFollowerMotorVelocity = indexerFollowerVelocity.getValue();
  }

  @Override
  public void applyOutputs(IndexerIOOutputs outputs) {
    switch (outputs.indexerMode) {
      case DUTY_CYCLE -> indexerLeader.setControl(
          indexerLeaderController.withOutput(outputs.targetDutyCycle));

      case STOP -> {
        indexerLeader.stopMotor();
        indexerFollower.stopMotor();

        indexerFollower.setControl(indexerFollowerController);
      }
    }
  }
}
