package com.stuypulse.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.stuypulse.robot.util.logged.LoggedTalonFX.TalonFXInputs;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public TalonFXInputs pivotInputs = new TalonFXInputs();

        public TalonFXInputs rollerLeftInputs = new TalonFXInputs();

        public TalonFXInputs rollerRightInputs = new TalonFXInputs();
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public static enum PivotIOOutputMode {
        POSITION,
        VOLTAGE,
        STOP
    }

    public static enum RollerIOOutputMode {
        DUTY_CYCLE,
        STOP
    }

    public static class IntakeIOOutputs {
        @AutoLogOutput(key = "Intake/Pivot/Output Mode")
        public PivotIOOutputMode pivotMode = PivotIOOutputMode.POSITION;

        @AutoLogOutput(key = "Intake/Pivot/Target Position")
        public Angle pivotTargetPosition = Degrees.zero();

        @AutoLogOutput(key = "Intake/Pivot/Target Voltage")
        public Voltage pivotTargetVoltage = Volts.zero();

        @AutoLogOutput(key = "Intake/Rollers/Output Mode")
        public RollerIOOutputMode rollerMode = RollerIOOutputMode.DUTY_CYCLE;

        @AutoLogOutput(key = "Intake/Rollers/Target Duty Cycle")
        public double rollerTargetDutyCycle = 0.0;
    }

    public default void applyOutputs(IntakeIOOutputs outputs) {}

    public default void seedPivotPosition(Angle position) {}
}
