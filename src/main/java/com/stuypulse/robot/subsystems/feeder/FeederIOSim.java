package com.stuypulse.robot.subsystems.feeder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.constants.GlobalSettings;
import com.stuypulse.robot.subsystems.feeder.FeederConstants.*;
import com.stuypulse.robot.util.simulation.TalonFXSimulation.SystemSim;
import com.stuypulse.robot.util.simulation.TalonFXSimulation.TalonFXSimulation;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class FeederIOSim implements FeederIO {
  private final SystemSim<DCMotorSim> feederSystem;

  private final TalonFXSimulation feederLeader;
  private final TalonFXSimulation feederFollower;

  private final DutyCycleOut feederLeaderController;
  private final Follower feederFollowerController;

  private final StatusSignal<Angle> feederLeaderPosition;
  private final StatusSignal<Current> feederLeaderSupplyCurrent;
  private final StatusSignal<Current> feederLeaderStatorCurrent;
  private final StatusSignal<Temperature> feederLeaderTemperature;
  private final StatusSignal<Voltage> feederLeaderAppliedVoltage;
  private final StatusSignal<AngularVelocity> feederLeaderVelocity;

  private final StatusSignal<Angle> feederFollowerPosition;
  private final StatusSignal<Current> feederFollowerSupplyCurrent;
  private final StatusSignal<Current> feederFollowerStatorCurrent;
  private final StatusSignal<Temperature> feederFollowerTemperature;
  private final StatusSignal<Voltage> feederFollowerAppliedVoltage;
  private final StatusSignal<AngularVelocity> feederFollowerVelocity;

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

    feederLeaderPosition = feederLeader.getPosition();
    feederLeaderSupplyCurrent = feederLeader.getSupplyCurrent();
    feederLeaderStatorCurrent = feederLeader.getStatorCurrent();
    feederLeaderTemperature = feederLeader.getDeviceTemp();
    feederLeaderAppliedVoltage = feederLeader.getMotorVoltage();
    feederLeaderVelocity = feederLeader.getVelocity();

    feederFollowerPosition = feederFollower.getPosition();
    feederFollowerSupplyCurrent = feederFollower.getSupplyCurrent();
    feederFollowerStatorCurrent = feederFollower.getStatorCurrent();
    feederFollowerTemperature = feederFollower.getDeviceTemp();
    feederFollowerAppliedVoltage = feederFollower.getMotorVoltage();
    feederFollowerVelocity = feederFollower.getVelocity();
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    feederSystem.update(GlobalSettings.DT);
    feederLeader.refresh();
    feederFollower.refresh();

    BaseStatusSignal.refreshAll(
        feederLeaderPosition,
        feederLeaderSupplyCurrent,
        feederLeaderStatorCurrent,
        feederLeaderTemperature,
        feederLeaderAppliedVoltage,
        feederLeaderVelocity,
        feederFollowerPosition,
        feederFollowerSupplyCurrent,
        feederFollowerStatorCurrent,
        feederFollowerTemperature,
        feederFollowerAppliedVoltage,
        feederFollowerVelocity);

    inputs.feederLeaderMotorPosition = feederLeaderPosition.getValue();
    inputs.feederLeaderMotorSupplyCurrent = feederLeaderSupplyCurrent.getValue();
    inputs.feederLeaderMotorStatorCurrent = feederLeaderStatorCurrent.getValue();
    inputs.feederLeaderMotorTemperature = feederLeaderTemperature.getValue();
    inputs.feederLeaderMotorAppliedVoltage = feederLeaderAppliedVoltage.getValue();
    inputs.feederLeaderMotorVelocity = feederLeaderVelocity.getValue();

    inputs.feederFollowerMotorPosition = feederFollowerPosition.getValue();
    inputs.feederFollowerMotorSupplyCurrent = feederFollowerSupplyCurrent.getValue();
    inputs.feederFollowerMotorStatorCurrent = feederFollowerStatorCurrent.getValue();
    inputs.feederFollowerMotorTemperature = feederFollowerTemperature.getValue();
    inputs.feederFollowerMotorAppliedVoltage = feederFollowerAppliedVoltage.getValue();
    inputs.feederFollowerMotorVelocity = feederFollowerVelocity.getValue();
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
