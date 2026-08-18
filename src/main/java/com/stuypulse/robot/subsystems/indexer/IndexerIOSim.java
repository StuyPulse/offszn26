package com.stuypulse.robot.subsystems.indexer;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.constants.GlobalSettings;
import com.stuypulse.robot.subsystems.indexer.IndexerConstants.*;
import com.stuypulse.robot.util.simulation.TalonFXSimulation.SystemSim;
import com.stuypulse.robot.util.simulation.TalonFXSimulation.TalonFXSimulation;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IndexerIOSim implements IndexerIO {
  private final SystemSim<DCMotorSim> indexerSystem;

  private final TalonFXSimulation indexerLeader;
  private final TalonFXSimulation indexerFollower;

  private final DutyCycleOut indexerLeaderController;
  private final Follower indexerFollowerController;

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
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    indexerSystem.update(GlobalSettings.DT);
    indexerLeader.refresh();
    indexerFollower.refresh();

    indexerLeader.updateInputs(inputs.indexerLeaderInputs);
    indexerFollower.updateInputs(inputs.indexerFollowerInputs);
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
