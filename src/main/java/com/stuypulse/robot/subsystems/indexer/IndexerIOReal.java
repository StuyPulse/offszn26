package com.stuypulse.robot.subsystems.indexer;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.constants.GlobalSettings;
import com.stuypulse.robot.subsystems.indexer.IndexerConstants.*;
import com.stuypulse.robot.util.logged.LoggedTalonFX.LoggedTalonFX;

public class IndexerIOReal implements IndexerIO {
  private final LoggedTalonFX indexerBackMotor;
  private final LoggedTalonFX indexerFrontMotor;

  private final DutyCycleOut indexerLeaderController;
  private final Follower indexerFollowerController;

  public IndexerIOReal() {
    indexerBackMotor = new LoggedTalonFX(IndexerDeviceIds.LEADER, GlobalSettings.RIO);
    indexerFrontMotor = new LoggedTalonFX(IndexerDeviceIds.FOLLOWER, GlobalSettings.RIO);

    IndexerMotorConfigs.INDEXER_CONFIG.configure(indexerBackMotor);
    IndexerMotorConfigs.INDEXER_CONFIG.configure(indexerFrontMotor);

    indexerLeaderController = new DutyCycleOut(0).withEnableFOC(true);
    indexerFollowerController =
        new Follower(indexerBackMotor.getDeviceID(), MotorAlignmentValue.Opposed);

    indexerFrontMotor.setControl(indexerFollowerController);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
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
