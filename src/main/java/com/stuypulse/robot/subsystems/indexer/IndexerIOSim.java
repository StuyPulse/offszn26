package com.stuypulse.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.constants.GlobalSettings;
import com.stuypulse.robot.subsystems.indexer.IndexerConstants.*;
import com.stuypulse.robot.util.simulation.TalonFXSimulation.SystemSim;
import com.stuypulse.robot.util.simulation.TalonFXSimulation.TalonFXSimulation;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IndexerIOSim implements IndexerIO {
  private final SystemSim<DCMotorSim> indexerSystem;

  private final TalonFXSimulation indexerLeader;
  private final TalonFXSimulation indexerFollower;

  private final DutyCycleOut indexerLeaderController;
  private final Follower indexerFollowerController;

  private final StatusSignal<Current> indexerLeaderSupplyCurrent;
  private final StatusSignal<Current> indexerLeaderStatorCurrent;
  private final StatusSignal<Temperature> indexerLeaderTemperature;
  private final StatusSignal<Voltage> indexerLeaderAppliedVoltage;
  private final StatusSignal<AngularVelocity> indexerLeaderVelocity;

  private final StatusSignal<Angle> indexerFollowerPosition;
  private final StatusSignal<Current> indexerFollowerSupplyCurrent;
  private final StatusSignal<Current> indexerFollowerStatorCurrent;
  private final StatusSignal<Temperature> indexerFollowerTemperature;
  private final StatusSignal<Voltage> indexerFollowerAppliedVoltage;
  private final StatusSignal<AngularVelocity> indexerFollowerVelocity;

  public IndexerIOSim() {
    indexerSystem =
        SystemSim.of(
            new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                    DCMotor.getKrakenX60Foc(2), 0.01, IndexerSettings.GEAR_RATIO),
                DCMotor.getKrakenX60Foc(2)));

    indexerLeader =
        new TalonFXSimulation(IndexerDeviceIds.LEADER, IndexerSettings.GEAR_RATIO, indexerSystem);
    indexerFollower =
        new TalonFXSimulation(IndexerDeviceIds.FOLLOWER, IndexerSettings.GEAR_RATIO, indexerSystem);

    indexerLeader.configure(IndexerMotorConfigs.INDEXER_CONFIG);
    indexerFollower.configure(IndexerMotorConfigs.INDEXER_CONFIG);

    indexerFollower.linkToReference(indexerLeader);

    indexerLeaderController = new DutyCycleOut(0).withEnableFOC(true);
    indexerFollowerController =
        new Follower(indexerLeader.getDeviceID(), MotorAlignmentValue.Opposed);

    indexerFollower.setControl(indexerFollowerController);

    indexerLeaderSupplyCurrent = indexerLeader.getSupplyCurrent();
    indexerLeaderStatorCurrent = indexerLeader.getStatorCurrent();
    indexerLeaderTemperature = indexerLeader.getDeviceTemp();
    indexerLeaderAppliedVoltage = indexerLeader.getMotorVoltage();
    indexerLeaderVelocity = indexerLeader.getVelocity();

    indexerFollowerPosition = indexerFollower.getPosition();
    indexerFollowerSupplyCurrent = indexerFollower.getSupplyCurrent();
    indexerFollowerStatorCurrent = indexerFollower.getStatorCurrent();
    indexerFollowerTemperature = indexerFollower.getDeviceTemp();
    indexerFollowerAppliedVoltage = indexerFollower.getMotorVoltage();
    indexerFollowerVelocity = indexerFollower.getVelocity();
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    indexerSystem.update(GlobalSettings.DT);
    indexerLeader.refresh();
    indexerFollower.refresh();

    BaseStatusSignal.refreshAll(
        indexerLeaderSupplyCurrent,
        indexerLeaderStatorCurrent,
        indexerLeaderTemperature,
        indexerLeaderAppliedVoltage,
        indexerLeaderVelocity,
        indexerFollowerPosition,
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