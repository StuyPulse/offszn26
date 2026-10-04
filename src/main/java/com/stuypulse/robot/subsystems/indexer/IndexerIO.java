package com.stuypulse.robot.subsystems.indexer;

import com.stuypulse.robot.util.logged.LoggedTalonFX.TalonFXInputs;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

public interface IndexerIO {
    @AutoLog
    public class IndexerIOInputs {
        public TalonFXInputs indexerBackInputs = new TalonFXInputs();
        public TalonFXInputs indexerFrontInputs = new TalonFXInputs();
    }

    public default void updateInputs(IndexerIOInputs inputs) {}

    public enum IndexerIOOutputMode {
        DUTY_CYCLE,
        STOP
    }

    public class IndexerIOOutputs {
        @AutoLogOutput(key = "Indexer/Output Mode")
        public IndexerIOOutputMode indexerMode = IndexerIOOutputMode.STOP;

        @AutoLogOutput(key = "Indexer/Target Duty Cycle")
        public double targetDutyCycle = 0;
    }

    public default void applyOutputs(IndexerIOOutputs outputs) {}
}
