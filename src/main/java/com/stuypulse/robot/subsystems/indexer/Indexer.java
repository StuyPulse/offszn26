package com.stuypulse.robot.subsystems.indexer;

import com.stuypulse.robot.constants.GlobalSettings;
import com.stuypulse.robot.subsystems.indexer.IndexerConstants.IndexerSettings;
import com.stuypulse.robot.subsystems.indexer.IndexerIO.IndexerIOOutputMode;
import com.stuypulse.robot.subsystems.indexer.IndexerIO.IndexerIOOutputs;
import com.stuypulse.robot.util.FullSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Indexer extends FullSubsystem {
    private final IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs;
    private final IndexerIOOutputs outputs;

    @AutoLogOutput(key = "Indexer/State")
    private IndexerState state;

    public Indexer(IndexerIO io) {
        this.io = io;
        inputs = new IndexerIOInputsAutoLogged();
        outputs = new IndexerIOOutputs();

        setState(IndexerState.STOP);
    }

    public enum IndexerState {
        FORWARD,
        REVERSE,
        STOP
    }

    private void setState(IndexerState state) {
        this.state = state;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);

        if (!GlobalSettings.EnabledSubsystems.INDEXER.get()) {
            stopMotor();

            return;
        }

        switch (state) {
            case FORWARD -> runDutyCycle(IndexerSettings.FORWARD_DUTY_CYCLE);

            case REVERSE -> runDutyCycle(IndexerSettings.REVERSE_DUTY_CYCLE);

            case STOP -> stopMotor();
        }
    }

    @Override
    public void periodicAfterScheduler() {
        io.applyOutputs(outputs);
    }

    private void runDutyCycle(double dutyCycle) {
        outputs.indexerMode = IndexerIOOutputMode.DUTY_CYCLE;
        outputs.targetDutyCycle = dutyCycle;
    }

    private void stopMotor() {
        outputs.indexerMode = IndexerIOOutputMode.STOP;
    }

    public Command runForward() {
        return runOnce(() -> setState(IndexerState.FORWARD)).withName("Indexer Forward");
    }

    public Command runReverse() {
        return runOnce(() -> setState(IndexerState.REVERSE)).withName("Indexer Reverse");
    }

    public Command stop() {
        return runOnce(() -> setState(IndexerState.STOP)).withName("Indexer Stop");
    }
}
