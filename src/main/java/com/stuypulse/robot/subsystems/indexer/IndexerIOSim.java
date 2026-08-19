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

  private final TalonFXSimulation indexerBackMotor;
  private final TalonFXSimulation indexerFrontMotor;

  private final DutyCycleOut indexerLeaderController;
  private final Follower indexerFollowerController;

  public IndexerIOSim() {
    indexerSystem =
        SystemSim.of(
            new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                    DCMotor.getKrakenX60Foc(2), 0.01, IndexerSettings.GEAR_RATIO),
                DCMotor.getKrakenX60Foc(2)));

    indexerBackMotor =
        new TalonFXSimulation(IndexerDeviceIds.LEADER, IndexerSettings.GEAR_RATIO, indexerSystem);
    indexerFrontMotor =
        new TalonFXSimulation(IndexerDeviceIds.FOLLOWER, IndexerSettings.GEAR_RATIO, indexerSystem);

    indexerBackMotor.configure(IndexerMotorConfigs.INDEXER_CONFIG);
    indexerFrontMotor.configure(IndexerMotorConfigs.INDEXER_CONFIG);

    indexerFrontMotor.linkToReference(indexerBackMotor);

    indexerLeaderController = new DutyCycleOut(0).withEnableFOC(true);
    indexerFollowerController =
        new Follower(indexerBackMotor.getDeviceID(), MotorAlignmentValue.Opposed);

    indexerFrontMotor.setControl(indexerFollowerController);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    indexerSystem.update(GlobalSettings.DT);
    indexerBackMotor.refresh();
    indexerFrontMotor.refresh();

    indexerBackMotor.updateInputs(inputs.indexerBackInputs);
    indexerFrontMotor.updateInputs(inputs.indexerFrontInputs);
  }

  @Override
  public void applyOutputs(IndexerIOOutputs outputs) {
    switch (outputs.indexerMode) {
      case DUTY_CYCLE -> indexerBackMotor.setControl(
          indexerLeaderController.withOutput(outputs.targetDutyCycle));

      case STOP -> {
        indexerBackMotor.stopMotor();
        indexerFrontMotor.stopMotor();

        indexerFrontMotor.setControl(indexerFollowerController);
      }
    }
  }
}
