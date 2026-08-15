// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package com.stuypulse.robot.commands;

import static edu.wpi.first.units.Units.*;

import com.stuypulse.robot.constants.DriverConstants.*;
import com.stuypulse.robot.subsystems.swerve.SwerveConstants.*;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.swerve.Swerve;
import com.stuypulse.robot.util.swerve.AlignmentUtil;
import com.stuypulse.robot.util.swerve.DriveInputProcessor;
import com.stuypulse.robot.util.swerve.DriveTurnInputProcessor;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;

public class DriveCommands {
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  private DriveCommands() {}

  public static Command buzzController(CommandXboxController driver) {
    return Commands.run(
            () -> {
              driver.getHID().setRumble(RumbleType.kBothRumble, Driver.BUZZ_INTENSITY);
            })
        .withName("Buzz Controller");
  }

  public static Command resetHeading(Swerve swerve) {
    return Commands.runOnce(
            () -> {
              swerve.resetHeading(Rotation2d.kZero);
            },
            swerve)
        .withName("Reset Heading");
  }

  public static Command resetPose(Swerve swerve, Pose2d pose) {
    return Commands.runOnce(
        () -> {
          swerve.resetOdometry(pose);
        },
        swerve);
  }

  public static Command xMode(Swerve swerve) {
    return Commands.run(
            () -> {
              swerve.stopWithX();
            })
        .withName("Swerve X Mode");
  }

  /**
   * Field relative swerve command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(Swerve swerve, CommandXboxController driver) {
    DriveInputProcessor driveInputProcessor =
        new DriveInputProcessor(
            driver,
            Driver.Drive.DEADBAND,
            Driver.Drive.POWER,
            DriveConstraints.MAX_VELOCITY,
            DriveConstraints.MAX_ACCEL,
            Driver.Drive.RC);
    DriveTurnInputProcessor driveTurnInputProcessor =
        new DriveTurnInputProcessor(
            driver,
            Driver.Turn.DEADBAND,
            Driver.Turn.POWER,
            DriveConstraints.MAX_ANGULAR_VEL,
            Driver.Turn.RC);

    return Commands.run(
            () -> {
              driveInputProcessor.update();
              driveTurnInputProcessor.update();
              // Get linear velocity
              Translation2d linearVelocity = driveInputProcessor.get();

              // Get angular velocity
              AngularVelocity angularVelocity = driveTurnInputProcessor.get();

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX(),
                      linearVelocity.getY(),
                      angularVelocity.in(RadiansPerSecond));
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              swerve.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? swerve.getRotation().plus(new Rotation2d(Math.PI))
                          : swerve.getRotation()));
            },
            swerve)
        .withName("Drive");
  }

  public static Command alignToPose(Swerve swerve, Pose2d targetPose) {
    PIDController angleController =
        new PIDController(SwerveSettings.Alignment.Gains.kP, SwerveSettings.Alignment.Gains.kI, SwerveSettings.Alignment.Gains.kD);
    Debouncer isAlignedDebouncer =
        new Debouncer(SwerveSettings.Alignment.IS_ALIGNED_DEBOUNCE.in(Seconds), DebounceType.kBoth);

    angleController.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.runEnd(
            () -> {
              Rotation2d targetHeading =
                  AlignmentUtil.getTargetAlignmentAngle(swerve.getPose(), targetPose);

              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      0,
                      0,
                      angleController.calculate(
                          swerve.getRotation().getRadians(), targetHeading.getRadians()));

              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              swerve.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? swerve.getRotation().plus(new Rotation2d(Math.PI))
                          : swerve.getRotation()));
            },
            () -> angleController.close(),
            swerve)
        .until(
            () ->
                isAlignedDebouncer.calculate(
                    Math.abs(angleController.getError()) < SwerveSettings.Alignment.THETA_TOLERANCE.getRadians()))
        .withName("Swerve Align To Pose");
  }

  public static Command alignToHub(Swerve swerve) {
    return alignToPose(swerve, Field.HUB_CENTER).withName("Swerve Align To Hub");
  }

  public static Command alignToFerryZone(Swerve swerve) {
    return alignToPose(swerve, Field.getFerryZonePose(swerve.getPose().getTranslation()))
        .withName("Swerve Align To Ferry Zone");
  }

  /**
   * Measures the velocity feedforward constants for the swerve motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Swerve swerve) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(() -> swerve.runCharacterization(0.0), swerve).withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  swerve.runCharacterization(voltage);
                  velocitySamples.add(swerve.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                swerve)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Swerve swerve) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                  swerve.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                },
                swerve)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = swerve.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = swerve.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = swerve.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = swerve.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius =
                          (state.gyroDelta * Swerve.DRIVE_BASE_RADIUS) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }
}
