package com.stuypulse.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.stuypulse.robot.util.config.TalonFXConfig;
import edu.wpi.first.units.measure.*;

public interface ShooterConstants {
    public interface ShooterSettings {
        // dervied directly from the cad from the teeth of the pullies, but not confirmed
        // only used the right side, not sure if left is the same
        double LEFT_GEARING = 30.0 / 48.0;
        double RIGHT_GEARING = 30.0 / 48.0;

        // derived from the cad, but not confirmed
        Mass DRUM_MASS = Pounds.of(6.05);
        Distance DRUM_RADIUS = Inches.of(2.0);
        MomentOfInertia SHOOTER_MOI =
                KilogramSquareMeters.ofBaseUnits(
                        DRUM_MASS
                                .times(DRUM_RADIUS)
                                .times(DRUM_RADIUS)
                                .times(0.5)
                                .baseUnitMagnitude()); // I = 1/2 * m * r^2

        // TODO: Needs tuning
        AngularVelocity TOLERANCE = RPM.of(100);
    }

    public interface ShooterIds { // placeholders
        int TOP_LEFT_MOTOR = 1;
        int BOTTOM_LEFT_MOTOR = 2;
        int TOP_RIGHT_MOTOR = 3;
        int BOTTOM_RIGHT_MOTOR = 4;
    }

    public interface ShooterGains { // placeholders
        // pid
        double kP = 0.1;
        double kI = 0.0;
        double kD = 0.0;

        // ff
        double kS = 0.0;
        double kV = 0.0;
        double kA = 0.0;
    }

    public interface ShooterMotorConfigs {
        final TalonFXConfig LEFT_CONFIG =
                new TalonFXConfig()
                        .withNeutralMode(NeutralModeValue.Coast)
                        .withSupplyCurrentLimitEnabled(true)
                        .withSupplyCurrentLimitAmps(50.0) // placeholder
                        .withStatorCurrentLimitEnabled(true)
                        .withStatorCurrentLimitAmps(100.0) // placeholder
                        .withLowerLimitSupplyCurrent(40.0, 0.25) // placeholder
                        .withPIDConstants(ShooterGains.kP, ShooterGains.kI, ShooterGains.kD, 0)
                        .withFFConstants(ShooterGains.kS, ShooterGains.kV, ShooterGains.kA, 0)
                        .withSensorToMechanismRatio(ShooterSettings.LEFT_GEARING);

        final TalonFXConfig RIGHT_CONFIG =
                new TalonFXConfig()
                        .withNeutralMode(NeutralModeValue.Coast)
                        .withSupplyCurrentLimitEnabled(true)
                        .withSupplyCurrentLimitAmps(50.0) // placeholder
                        .withStatorCurrentLimitEnabled(true)
                        .withStatorCurrentLimitAmps(100.0) // placeholder
                        .withLowerLimitSupplyCurrent(40.0, 0.25) // placeholder
                        .withPIDConstants(ShooterGains.kP, ShooterGains.kI, ShooterGains.kD, 0)
                        .withFFConstants(ShooterGains.kS, ShooterGains.kV, ShooterGains.kA, 0)
                        .withInvertedValue(InvertedValue.Clockwise_Positive) // placeholder
                        .withSensorToMechanismRatio(ShooterSettings.RIGHT_GEARING);
    }
}
