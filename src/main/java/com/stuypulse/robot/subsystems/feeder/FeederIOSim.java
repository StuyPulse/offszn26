package com.stuypulse.robot.subsystems.feeder;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.constants.GlobalSettings;
import com.stuypulse.robot.subsystems.feeder.FeederConstants.*;
import com.stuypulse.robot.util.simulation.TalonFXSimulation.SystemSim;
import com.stuypulse.robot.util.simulation.TalonFXSimulation.TalonFXSimulation;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class FeederIOSim implements FeederIO {
  private final SystemSim<DCMotorSim> feederSystem;

  private final TalonFXSimulation feederLeader;
  private final TalonFXSimulation feederFollower;

  private final DutyCycleOut feederLeaderController;
  private final Follower feederFollowerController;

  public FeederIOSim() {
    feederSystem =
        SystemSim.of(
            new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                    DCMotor.getKrakenX60Foc(2), 0.01, FeederSettings.GEAR_RATIO),
                DCMotor.getKrakenX60Foc(2)));

    feederLeader =
        new TalonFXSimulation(FeederDeviceIds.LEADER, FeederSettings.GEAR_RATIO, feederSystem);
    feederFollower =
        new TalonFXSimulation(FeederDeviceIds.FOLLOWER, FeederSettings.GEAR_RATIO, feederSystem);

    feederLeader.configure(FeederMotorConfigs.FEEDER_CONFIG);
    feederFollower.configure(FeederMotorConfigs.FEEDER_CONFIG);

    feederFollower.linkToReference(feederLeader);

    feederLeaderController = new DutyCycleOut(0).withEnableFOC(true);
    feederFollowerController =
        new Follower(feederLeader.getDeviceID(), MotorAlignmentValue.Opposed);

    feederFollower.setControl(feederFollowerController);
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    feederSystem.update(GlobalSettings.DT);
    feederLeader.refresh();
    feederFollower.refresh();

    feederLeader.updateInputs(inputs.feederLeaderInputs);
    feederFollower.updateInputs(inputs.feederFollowerInputs);
  }

  @Override
  public void applyOutputs(FeederIOOutputs outputs) {
    switch (outputs.feederMode) {
      case DUTY_CYCLE -> feederLeader.setControl(
          feederLeaderController.withOutput(outputs.targetDutyCycle));

      case STOP -> {
        feederLeader.stopMotor();
        feederFollower.stopMotor();

        feederFollower.setControl(feederFollowerController);
      }
    }
  }
}
