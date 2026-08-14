package com.stuypulse.robot.subsystems.hood;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.stuypulse.robot.util.config.TalonFXConfig;

public interface HoodConstants {

  // TODO: ALL CONSTANTS ARE PLACEHOLDERS

  public interface HoodSettings {
    double GEAR_RATIO = 5;
  }

  public interface HoodGains {
    double kP = 250.0;
    double kI = 0.0;
    double kD = 2.0;

    double kS = 0.25;
    double kV = 0.0;
    double kA = 0.0;
  }

  public interface HoodDeviceIds {
    int HOOD_MOTOR = 5;
  }

  public interface HoodMotorConfigs {
    TalonFXConfig HOOD_CONFIG =
        new TalonFXConfig()
            .withInvertedValue(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)
            .withSupplyCurrentLimitAmps(80.0)
            .withStatorCurrentLimitEnabled(false)
            .withRampRate(0.25)
            .withPIDConstants(HoodGains.kP, HoodGains.kI, HoodGains.kD, 0)
            .withFFConstants(HoodGains.kS, HoodGains.kV, HoodGains.kA, 0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign, 0)
            .withSensorToMechanismRatio(HoodSettings.GEAR_RATIO);
  }
}
