package com.stuypulse.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public interface IndexerIO {
    @AutoLog
    public static class IndexerIOInputs {
        public Current indexerLeaderMotorSupplyCurrent = Amps.zero();
        public Current indexerLeaderMotorStatorCurrent = Amps.zero();
        public Temperature indexerLeaderMotorTemperature = Celsius.zero();
        public Voltage indexerLeaderMotorAppliedVoltage = Volts.zero();
        public AngularVelocity indexerLeaderMotorVelocity = RPM.zero();

        public Current indexerFollowerMotorSupplyCurrent = Amps.zero();
        public Current indexerFollowerMotorStatorCurrent = Amps.zero();
        public Temperature indexerFollowerMotorTemperature = Celsius.zero();
        public Voltage indexerFollowerMotorAppliedVoltage = Volts.zero();
        public AngularVelocity indexerFollowerMotorVelocity = RPM.zero();
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
