package com.stuypulse.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;

import com.stuypulse.robot.util.logged.LoggedTalonFX.TalonFXInputs;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;

public interface HoodIO {
    @AutoLog
    public static class HoodIOInputs {
        public TalonFXInputs hoodInputs = new TalonFXInputs();
    }

    public default void updateInputs(HoodIOInputs inputs) {}

    public static enum HoodIOOutputMode {
        POSITION,
        VOLTAGE,
        STOP
    }

    public static class HoodIOOutputs {
        @AutoLogOutput(key = "Hood/Output Mode")
        public HoodIOOutputMode hoodMode = HoodIOOutputMode.POSITION;

        @AutoLogOutput(key = "Hood/Target Position")
        public Angle hoodTargetPosition = Degrees.zero();

        @AutoLogOutput(key = "Hood/Target Voltage")
        public Voltage hoodTargetVoltage = Volts.zero();
    }

    public default void applyOutputs(HoodIOOutputs outputs) {}

    public default void seedPosition(Angle position) {}
}
