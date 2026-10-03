package com.stuypulse.robot.subsystems.feeder;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.stuypulse.robot.util.config.TalonFXConfig;

public interface FeederConstants {
    public interface FeederSettings {
        // TODO: Placeholders

        double FORWARD_DUTY_CYCLE = 1;
        double REVERSE_DUTY_CYCLE = -1;

        // Sim
        double GEAR_RATIO = 3;
    }

    public interface FeederDeviceIds {
        // TODO: Placeholders

        int LEADER = 12;

        int FOLLOWER = 13;
    }

    public interface FeederMotorConfigs {
        TalonFXConfig FEEDER_CONFIG =
                new TalonFXConfig()
                        .withInvertedValue(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Brake);
    }
}
