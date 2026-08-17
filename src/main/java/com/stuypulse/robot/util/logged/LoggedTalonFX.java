package com.stuypulse.robot.util.logged;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.*;

/**
 * <h2>Logged TalonFX</h2>
 *
 * <p>
 * A {@link TalonFX} motor controller with built-in signal logging for
 * AdvantageKit.
 *
 * <p>
 * This class wraps the commonly used TalonFX status signals and provides a
 * convenient method for refreshing those signals and copying their values into
 * a {@link TalonFXInputs} object.
 *
 * <h3>Usage</h3>
 *
 * <p>
 * Include a {@link TalonFXInputsAutoLogged} field for each motor that should
 * have its telemetry logged.
 *
 * 
 * <pre>{@code
 * @AutoLog
 * public static class IntakeIOInputs {
 *     public TalonFXInputsAutoLogged pivotSignals = new TalonFXInputsAutoLogged();
 *
 *     public TalonFXInputsAutoLogged rollerSignals = new TalonFXInputsAutoLogged();
 * }
 * }</pre>
 *
 * <h3>Creating the Motor</h3>
 *
 * <p>
 * Use {@code LoggedTalonFX} anywhere a normal {@link TalonFX} would be used:
 *
 * <pre>{@code
 * private final LoggedTalonFX pivotMotor;
 *
 * public IntakeIOReal() {
 *     pivotMotor = new LoggedTalonFX(
 *             IntakeDeviceIds.PIVOT,
 *             GlobalSettings.RIO);
 * }
 * }</pre>
 *
 * <h3>Updating Inputs</h3>
 *
 * <p>
 * Call {@link #updateInputs(TalonFXInputs)} from the subsystem's IO
 * implementation to refresh the motor telemetry:
 *
 * <pre>{@code
 * @Override
 * public void updateInputs(IntakeIOInputs inputs) {
 *     pivotMotor.updateInputs(inputs.pivotSignals);
 *     rollerMotor.updateInputs(inputs.rollerSignals);
 * }
 * }</pre>
 *
 * @see TalonFXInputs
 * @author Faizaan Jamil (https://github.com/Faizaan-J)
 */
public class LoggedTalonFX extends TalonFX {
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> statorCurrent;
    private final StatusSignal<Temperature> temperature;
    private final StatusSignal<Angle> position;
    private final StatusSignal<Voltage> appliedVoltage;
    private final StatusSignal<AngularVelocity> velocity;

    public LoggedTalonFX(int deviceId, CANBus bus) {
        super(deviceId, bus);
        supplyCurrent = getSupplyCurrent();
        statorCurrent = getStatorCurrent();
        temperature = getDeviceTemp();
        position = getPosition();
        appliedVoltage = getMotorVoltage();
        velocity = getVelocity();
    }

    public LoggedTalonFX(int deviceId) {
        this(deviceId, new CANBus());
    }

    /**
     * Refreshes the motor's status signals and copies their values into the
     * provided {@link TalonFXInputs} object.
     * @param inputs The {@link TalonFXInputs} object to update.
     */
    public void updateInputs(TalonFXInputs inputs) {
        BaseStatusSignal.refreshAll(
                supplyCurrent,
                statorCurrent,
                temperature,
                position,
                appliedVoltage,
                velocity);

        inputs.supplyCurrent = supplyCurrent.getValue();
        inputs.statorCurrent = statorCurrent.getValue();
        inputs.temperature = temperature.getValue();
        inputs.position = position.getValue();
        inputs.appliedVoltage = appliedVoltage.getValue();
        inputs.velocity = velocity.getValue();
    }
}
