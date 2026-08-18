package com.stuypulse.robot.subsystems.indexer;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.constants.GlobalSettings;
import com.stuypulse.robot.subsystems.indexer.IndexerConstants.*;
import com.stuypulse.robot.util.logged.LoggedTalonFX.LoggedTalonFX;

public class IndexerIOReal implements IndexerIO {
  private final LoggedTalonFX indexerLeader;
  private final LoggedTalonFX indexerFollower;

  private final DutyCycleOut indexerLeaderController;
  private final Follower indexerFollowerController;

  public IndexerIOReal() {
    indexerLeader = new LoggedTalonFX(IndexerDeviceIds.LEADER, GlobalSettings.RIO);
    indexerFollower = new LoggedTalonFX(IndexerDeviceIds.FOLLOWER, GlobalSettings.RIO);

    IndexerMotorConfigs.INDEXER_CONFIG.configure(indexerLeader);
    IndexerMotorConfigs.INDEXER_CONFIG.configure(indexerFollower);

    indexerLeaderController = new DutyCycleOut(0).withEnableFOC(true);
    indexerFollowerController =
        new Follower(indexerLeader.getDeviceID(), MotorAlignmentValue.Opposed);

    indexerFollower.setControl(indexerFollowerController);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
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
