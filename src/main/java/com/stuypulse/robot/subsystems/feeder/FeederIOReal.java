package com.stuypulse.robot.subsystems.feeder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.constants.GlobalSettings;
import com.stuypulse.robot.subsystems.feeder.FeederConstants.*;
import edu.wpi.first.units.measure.*;

public class FeederIOReal implements FeederIO {
  private final TalonFX feederLeader;
  private final TalonFX feederFollower;

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

  public FeederIOReal() {
    feederLeader = new TalonFX(FeederDeviceIds.LEADER, GlobalSettings.RIO);
    feederFollower = new TalonFX(FeederDeviceIds.FOLLOWER, GlobalSettings.RIO);

    FeederMotorConfigs.FEEDER_CONFIG.configure(feederLeader);
    FeederMotorConfigs.FEEDER_CONFIG.configure(feederFollower);

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
