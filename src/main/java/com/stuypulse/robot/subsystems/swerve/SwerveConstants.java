package com.stuypulse.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Time;

public interface SwerveConstants {
    public interface SwerveSettings {
        public interface Alignment {
            // TODO: Placeholder values, needs tuning
            Rotation2d THETA_TOLERANCE = Rotation2d.fromDegrees(3.0);
            Rotation2d SHOOTING_TOLERANCE = Rotation2d.fromDegrees(10);

            Time IS_ALIGNED_DEBOUNCE = Seconds.of(0.5);

            public interface Gains {
                double kP = 20;

                double kI = 0;

                double kD = 5;
            }
        }
    }
}
