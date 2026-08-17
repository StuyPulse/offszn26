package com.stuypulse.robot.util.logged;

import java.lang.Cloneable;
import java.lang.Override;
import java.util.ArrayList;
import java.util.List;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * <h2>TalonFX Inputs</h2>
 *
 * <p>
 * Contains logged data collected from a {@link LoggedTalonFX}.
 *
 * <p>
 * This class is annotated with {@link AutoLog} so AdvantageKit can
 * automatically generate the corresponding logged inputs class and record
 * the motor signals.
 *
 * <p>
 * A {@code TalonFXInputs} instance is typically stored as a field in a
 * subsystem's {@code IOInputs} class:
 *
 * <pre>{@code
 * @AutoLog
 * public static class IntakeIOInputs {
 *     public TalonFXInputsAutoLogged pivotSignals = new TalonFXInputsAutoLogged();
 *     public TalonFXInputsAutoLogged rollerSignals = new TalonFXInputsAutoLogged();
 * }
 * }</pre>
 *
 * <p>
 * The values are populated by
 * {@link LoggedTalonFX#updateInputs(TalonFXInputs)} and should not be
 * updated directly by the subsystem. Any signals registered on the owning
 * {@link LoggedTalonFX} via {@code addSignal} are logged and replayed
 * automatically — see {@link LoggedSignal}.
 *
 * @see LoggedTalonFX
 * @see LoggedSignal
 * @author Faizaan (https://github.com/Faizaan-J)
 */
public class TalonFXInputs implements LoggableInputs, Cloneable {
    public Current supplyCurrent = Amps.zero();
    public Current statorCurrent = Amps.zero();
    public Temperature temperature = Celsius.zero();
    public Angle position = Degrees.zero();
    public Voltage appliedVoltage = Volts.zero();
    public AngularVelocity velocity = DegreesPerSecond.zero();

    private List<LoggedSignal<?>> additionalSignals = new ArrayList<>();

    /**
     * Updates the additional signals associated with this motor.
     *
     * @param signals The signals to log.
     */
    public void setAdditionalSignals(List<LoggedSignal<?>> signals) {
        this.additionalSignals = signals;
    }

    @Override
    public void toLog(LogTable table) {
        table.put("SupplyCurrent", supplyCurrent);
        table.put("StatorCurrent", statorCurrent);
        table.put("Temperature", temperature);
        table.put("Position", position);
        table.put("AppliedVoltage", appliedVoltage);
        table.put("Velocity", velocity);

        for (LoggedSignal<?> signal : additionalSignals) {
            signal.toLog(table);
        }
    }

    @Override
    public void fromLog(LogTable table) {
        supplyCurrent = table.get("SupplyCurrent", supplyCurrent);
        statorCurrent = table.get("StatorCurrent", statorCurrent);
        temperature = table.get("Temperature", temperature);
        position = table.get("Position", position);
        appliedVoltage = table.get("AppliedVoltage", appliedVoltage);
        velocity = table.get("Velocity", velocity);

        for (LoggedSignal<?> signal : additionalSignals) {
            signal.fromLog(table);
        }
    }

    public TalonFXInputs clone() {
        TalonFXInputs copy = new TalonFXInputs();
        copy.supplyCurrent = this.supplyCurrent;
        copy.statorCurrent = this.statorCurrent;
        copy.temperature = this.temperature;
        copy.position = this.position;
        copy.appliedVoltage = this.appliedVoltage;
        copy.velocity = this.velocity;
        copy.additionalSignals = new ArrayList<>(this.additionalSignals);
        return copy;
    }
}
