package com.stuypulse.robot.subsystems.indexer;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.stuypulse.robot.util.config.TalonFXConfig;

public interface IndexerConstants {
    
    //TODO: ALL VALUES ARE PLACE HOLDERS

    public interface IndexerSettings {

        double FORWARD_DUTY_CYCLE = 1;
        double REVERSE_DUTY_CYCLE = -1;

        double GEAR_RATIO = 0;

    }

    public interface IndexerDeviceIds {

        int LEADER = 1;
        int FOLLOWER = 2;
    }

    public interface IndexerMotorConfigs {

        TalonFXConfig INDEXER_CONFIG = 
            new TalonFXConfig()
                .withNeutralMode(NeutralModeValue.Brake)
                .withSupplyCurrentLimitAmps(0)
                .withStatorCurrentLimitEnabled(false)
                .withRampRate(0)
                .withPIDConstants(0, 0, 0, 0)
                .withFFConstants(0, 0, 0, 0);
    }
}
