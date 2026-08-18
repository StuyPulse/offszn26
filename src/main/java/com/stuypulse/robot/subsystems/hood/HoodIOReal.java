package com.stuypulse.robot.subsystems.hood;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.stuypulse.robot.constants.GlobalSettings;
import com.stuypulse.robot.subsystems.hood.HoodConstants.*;
import com.stuypulse.robot.util.logged.LoggedTalonFX;

public class HoodIOReal implements HoodIO {
  private final LoggedTalonFX hoodMotor;

  private final PositionVoltage positionController;

  public HoodIOReal() {
    hoodMotor = new LoggedTalonFX(HoodDeviceIds.HOOD_MOTOR, GlobalSettings.RIO);

    HoodMotorConfigs.HOOD_CONFIG.configure(hoodMotor);

    positionController = new PositionVoltage(0).withEnableFOC(true);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    hoodMotor.updateInputs(inputs.hoodInputs);
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
