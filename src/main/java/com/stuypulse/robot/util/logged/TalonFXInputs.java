package com.stuypulse.robot.util.logged;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

import org.littletonrobotics.junction.AutoLog;

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
 * The values are populated by {@link LoggedTalonFX#updateInputs(TalonFXInputs)}
 * and should not be updated directly by the subsystem.
 * 
 * @see LoggedTalonFX
 * @author Faizaan Jamil (https://github.com/Faizaan-J)
 */
@AutoLog
public class TalonFXInputs {
    public Current supplyCurrent = Amps.zero();
    public Current statorCurrent = Amps.zero();
    public Temperature temperature = Celsius.zero();
    public Angle position = Degrees.zero();
    public Voltage appliedVoltage = Volts.zero();
    public AngularVelocity velocity = DegreesPerSecond.zero();
}
