package com.stuypulse.robot.subsystems.indexer;

import com.stuypulse.robot.util.logged.LoggedTalonFX.TalonFXInputs;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

public interface IndexerIO {
    @AutoLog
    public static class IndexerIOInputs {
        public TalonFXInputs indexerBackInputs = new TalonFXInputs();
        public TalonFXInputs indexerFrontInputs = new TalonFXInputs();
    }

    public default void updateInputs(IndexerIOInputs inputs) {}

    public static enum IndexerIOOutputMode {
        DUTY_CYCLE,
        STOP
    }

    public static class IndexerIOOutputs {
        @AutoLogOutput(key = "Indexer/Output Mode")
        public IndexerIOOutputMode indexerMode = IndexerIOOutputMode.DUTY_CYCLE;

        @AutoLogOutput(key = "Indexer/Target Duty Cycle")
        public double targetDutyCycle = 0;
    }

    public default void applyOutputs(IndexerIOOutputs outputs) {}
}
