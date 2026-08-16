package com.stuypulse.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.stuypulse.robot.util.config.TalonFXConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public interface IntakeConstants {
  public interface IntakeSettings {
    // TODO: All placeholders, need to get real numbers

    // Pivot
    int PIVOT_REGULAR_GAIN_SLOT = 0;
    int PIVOT_DIGEST_GAIN_SLOT = 1;

    Angle PIVOT_STOW_ANGLE = Degrees.of(71.0);
    Angle PIVOT_DEPLOY_ANGLE = Degrees.of(-10.0);

    double PIVOT_GEAR_RATIO = 32.0 / 20.0 * 64.0 / 18.0 * 60.0 / 8.0;

    Angle PIVOT_MAX_ANGLE = Degrees.of(76.4);
    Angle PIVOT_MIN_ANGLE = Degrees.of(-10.0);

    Angle ROLLER_START_THRESHOLD = Degrees.of(10.0);
    Angle PIVOT_PUSHDOWN_THRESHOLD = Degrees.of(15.0);
    Current PIVOT_PUSHDOWN_CURRENT = Amps.of(-80.0);

    // Rollers
    double INTAKE_DUTY_CYCLE = 1;
    double OUTTAKE_DUTY_CYCLE = -1;

    // Sim
    Distance ARM_LENGTH = Meters.of(0.4);
    Mass ARM_MASS = Kilograms.of(2.0);
    MomentOfInertia PIVOT_MOI =
        KilogramSquareMeters.of(
            SingleJointedArmSim.estimateMOI(ARM_MASS.in(Kilograms), ARM_LENGTH.in(Meters)));
  }

  public interface IntakeDeviceIds {
    // TODO: Temporary ids, get real ones

    int PIVOT = 0;

    int ROLLER_LEADER = 1;

    int ROLLER_FOLLOWER = 2;
  }

  public interface IntakeMotorConfigs {
    // TODO: Placeholders

    TalonFXConfig PIVOT_CONFIG =
        new TalonFXConfig()
            .withInvertedValue(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)
            .withSupplyCurrentLimitAmps(10.0) // was 60 on practice day
            .withStatorCurrentLimitEnabled(false)
            .withRampRate(0.25)
            .withPIDConstants(IntakeGains.Pivot.kP, IntakeGains.Pivot.kI, IntakeGains.Pivot.kD, 0)
            .withFFConstants(
                IntakeGains.Pivot.kS,
                IntakeGains.Pivot.kV,
                IntakeGains.Pivot.kA,
                IntakeGains.Pivot.kG,
                0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign, 0)
            .withGravityType(GravityTypeValue.Arm_Cosine)
            .withSensorToMechanismRatio(IntakeSettings.PIVOT_GEAR_RATIO);

    TalonFXConfig ROLLER_CONFIG =
        new TalonFXConfig()
            .withInvertedValue(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast)
            .withSupplyCurrentLimitAmps(37.0)
            .withStatorCurrentLimitEnabled(false)
            .withRampRate(0.50);
  }

  public interface IntakeGains {
    // TODO: Placeholders, need tuning

    public interface Pivot {
      double kP = 125;
      double kI = 0;
      double kD = 10;

      double kS = 0;
      double kV = 0.12;
      double kA = 0;

      double kG = 0.5;
    }
  }
}
