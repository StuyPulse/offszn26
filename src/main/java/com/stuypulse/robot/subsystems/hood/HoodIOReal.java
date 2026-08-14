package com.stuypulse.robot.subsystems.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.GlobalSettings;
import com.stuypulse.robot.subsystems.hood.HoodConstants.*;
import edu.wpi.first.units.measure.*;

public class HoodIOReal implements HoodIO {
  private final TalonFX hoodMotor;

  private final PositionVoltage positionController;

  private final StatusSignal<Angle> hoodMotorPosition;
  private final StatusSignal<Current> hoodMotorSupplyCurrent;
  private final StatusSignal<Current> hoodMotorStatorCurrent;
  private final StatusSignal<Temperature> hoodMotorTemperature;
  private final StatusSignal<Voltage> hoodMotorAppliedVoltage;
  private final StatusSignal<AngularVelocity> hoodMotorVelocity;

  public HoodIOReal() {
    hoodMotor = new TalonFX(HoodDeviceIds.HOOD_MOTOR, GlobalSettings.RIO);

    HoodMotorConfigs.HOOD_CONFIG.configure(hoodMotor);

    positionController = new PositionVoltage(0).withEnableFOC(true);

    hoodMotorPosition = hoodMotor.getPosition();
    hoodMotorSupplyCurrent = hoodMotor.getSupplyCurrent();
    hoodMotorStatorCurrent = hoodMotor.getStatorCurrent();
    hoodMotorTemperature = hoodMotor.getDeviceTemp();
    hoodMotorAppliedVoltage = hoodMotor.getMotorVoltage();
    hoodMotorVelocity = hoodMotor.getVelocity();
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        hoodMotorPosition,
        hoodMotorSupplyCurrent,
        hoodMotorStatorCurrent,
        hoodMotorTemperature,
        hoodMotorAppliedVoltage,
        hoodMotorVelocity);

    inputs.hoodMotorPosition = hoodMotorPosition.getValue();
    inputs.hoodMotorSupplyCurrent = hoodMotorSupplyCurrent.getValue();
    inputs.hoodMotorStatorCurrent = hoodMotorStatorCurrent.getValue();
    inputs.hoodMotorTemperature = hoodMotorTemperature.getValue();
    inputs.hoodMotorAppliedVoltage = hoodMotorAppliedVoltage.getValue();
    inputs.hoodMotorVelocity = hoodMotorVelocity.getValue();
  }

  @Override
  public void applyOutputs(HoodIOOutputs outputs) {
    switch (outputs.hoodMode) {
      case POSITION -> hoodMotor.setControl(
          positionController.withPosition(outputs.hoodTargetPosition));

      case STOP -> hoodMotor.stopMotor();
    }
  }
}
