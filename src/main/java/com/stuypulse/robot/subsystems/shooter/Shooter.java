package com.stuypulse.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;

import com.stuypulse.robot.constants.GlobalSettings;
import com.stuypulse.robot.subsystems.shooter.ShooterConstants.ShooterSettings;
import com.stuypulse.robot.subsystems.shooter.ShooterIO.ShooterIOOutputs;
import com.stuypulse.robot.subsystems.shooter.ShooterIO.ShooterIOOutputMode;
import com.stuypulse.robot.util.FullSubsystem;
import com.stuypulse.robot.util.InterpolationCalculator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class Shooter extends FullSubsystem {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs;
  private final ShooterIOOutputs outputs;

  @AutoLogOutput(key = "Shooter/State")
  private ShooterState state;

  private final Supplier<Pose2d> poseSupplier;

  private boolean atTolerance;

  public enum ShooterState {
    STOP,
    SHOOT,
    FERRY
  }

  public Shooter(ShooterIO io, Supplier<Pose2d> poseSupplier) {
    this.state = ShooterState.STOP;
    this.io = io;
    this.inputs = new ShooterIOInputsAutoLogged();
    this.outputs = new ShooterIOOutputs();

    this.poseSupplier = poseSupplier;

    setState(ShooterState.SHOOT);
  }

  private void runVelocity(AngularVelocity targetVelocity) {
    outputs.targetVelocity = targetVelocity;
    outputs.mode = ShooterIOOutputMode.VELOCITY_TORQUE_CURRENT_FOC;

    atTolerance =
        inputs.topLeftMotorInputs.velocity.minus(targetVelocity).abs(RPM)
            < ShooterSettings.TOLERANCE.in(RPM);
  }

  private void setState(ShooterState state) {
    this.state = state;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    if (!GlobalSettings.EnabledSubsystems.SHOOTER.get()) {
      outputs.mode = ShooterIOOutputMode.STOP;
      return;
    }

    switch (state) {
      case STOP -> outputs.mode = ShooterIOOutputMode.STOP;
      case SHOOT -> runVelocity(InterpolationCalculator.getInterpolatedShotRPM(poseSupplier.get()));
      case FERRY -> runVelocity(
          InterpolationCalculator.getInterpolatedFerryRPM(poseSupplier.get()));
    }
  }

  @Override
  public void periodicAfterScheduler() {
    io.applyOutputs(outputs);
  }

  public boolean atTolerance() {
    return atTolerance;
  }

  public Command shoot() {
    return runOnce(() -> setState(ShooterState.SHOOT)).withName("Shooter Shoot");
  }

  public Command ferry() {
    return runOnce(() -> setState(ShooterState.FERRY)).withName("Shooter Ferry");
  }

  public Command stop() {
    return runOnce(() -> setState(ShooterState.STOP)).withName("Shooter Stop");
  }
}
