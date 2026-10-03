package com.stuypulse.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.stuypulse.robot.util.config.TalonFXConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public interface HoodConstants {

    // TODO: ALL CONSTANTS ARE PLACEHOLDERS

    public interface HoodSettings {
        double GEAR_RATIO = 5;

        Angle STOW_ANGLE = Degrees.of(0);

        Angle FERRY_ANGLE = Degrees.of(30);

        Angle TOLERANCE = Degrees.of(3);

        // TODO: GET ACTUAL ANGLE FOR KB AND TOWER
        Angle KB_ANGLE = Degrees.of(694);

        Angle TOWER_ANGLE = Degrees.of(516);

        Current STALL_CURRENT = Amps.of(30);

        Voltage HOMING_VOLTAGE = Volts.of(2);
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
