package com.stuypulse.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.constants.GlobalSettings;
import com.stuypulse.robot.subsystems.shooter.ShooterConstants.*;
import com.stuypulse.robot.util.simulation.TalonFXSimulation.SystemSim;
import com.stuypulse.robot.util.simulation.TalonFXSimulation.TalonFXSimulation;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public final class ShooterIOSim implements ShooterIO {
    private final SystemSim<FlywheelSim> leftFlywheelSim;
    private final TalonFXSimulation topLeftMotor;
    private final TalonFXSimulation bottomLeftMotor;

    private final SystemSim<FlywheelSim> rightFlywheelSim;
    private final TalonFXSimulation topRightMotor;
    private final TalonFXSimulation bottomRightMotor;

    private final Follower shooterFollowerLeft;
    private final Follower shooterFollowerRight;

    private final VelocityTorqueCurrentFOC shooterVelocityControl;

    public ShooterIOSim() {
        this.leftFlywheelSim =
                SystemSim.of(
                        new FlywheelSim(
                                LinearSystemId.createFlywheelSystem(
                                        DCMotor.getKrakenX60Foc(2),
                                        ShooterSettings.SHOOTER_MOI.in(KilogramSquareMeters),
                                        ShooterSettings.LEFT_GEARING),
                                DCMotor.getKrakenX60Foc(2),
                                ShooterSettings.LEFT_GEARING));

        this.rightFlywheelSim =
                SystemSim.of(
                        new FlywheelSim(
                                LinearSystemId.createFlywheelSystem(
                                        DCMotor.getKrakenX60Foc(2),
                                        ShooterSettings.SHOOTER_MOI.in(KilogramSquareMeters),
                                        ShooterSettings.RIGHT_GEARING),
                                DCMotor.getKrakenX60Foc(2),
                                ShooterSettings.RIGHT_GEARING));

        this.topLeftMotor =
                new TalonFXSimulation(
                        ShooterIds.TOP_LEFT_MOTOR, ShooterSettings.LEFT_GEARING, leftFlywheelSim);
        this.bottomLeftMotor =
                new TalonFXSimulation(
                        ShooterIds.BOTTOM_LEFT_MOTOR,
                        ShooterSettings.LEFT_GEARING,
                        leftFlywheelSim);
        ShooterMotorConfigs.LEFT_CONFIG.configure(topLeftMotor);
        ShooterMotorConfigs.LEFT_CONFIG.configure(bottomLeftMotor);
        bottomLeftMotor.linkToReference(topLeftMotor);

        this.topRightMotor =
                new TalonFXSimulation(
                        ShooterIds.TOP_RIGHT_MOTOR,
                        ShooterSettings.RIGHT_GEARING,
                        rightFlywheelSim);
        this.bottomRightMotor =
                new TalonFXSimulation(
                        ShooterIds.BOTTOM_RIGHT_MOTOR,
                        ShooterSettings.RIGHT_GEARING,
                        rightFlywheelSim);
        ShooterMotorConfigs.RIGHT_CONFIG.configure(topRightMotor);
        ShooterMotorConfigs.RIGHT_CONFIG.configure(bottomRightMotor);
        bottomRightMotor.linkToReference(topRightMotor);

        this.shooterFollowerLeft =
                new Follower(topLeftMotor.getDeviceID(), MotorAlignmentValue.Aligned);
        this.shooterFollowerRight =
                new Follower(topRightMotor.getDeviceID(), MotorAlignmentValue.Opposed);

        this.shooterVelocityControl = new VelocityTorqueCurrentFOC(0);

        bottomLeftMotor.setControl(shooterFollowerLeft);
        bottomRightMotor.setControl(shooterFollowerRight);
        topLeftMotor.setControl(shooterVelocityControl);
        topRightMotor.setControl(shooterVelocityControl);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        leftFlywheelSim.update(GlobalSettings.DT);
        topLeftMotor.refresh();
        bottomRightMotor.refresh();

        rightFlywheelSim.update(GlobalSettings.DT);
        topRightMotor.refresh();
        bottomLeftMotor.refresh();

        topLeftMotor.updateInputs(inputs.topLeftMotorInputs);
        bottomLeftMotor.updateInputs(inputs.bottomLeftMotorInputs);
        topRightMotor.updateInputs(inputs.topRightMotorInputs);
        bottomRightMotor.updateInputs(inputs.bottomRightMotorInputs);
    }

    @Override
    public void applyOutputs(ShooterIOOutputs outputs) {
        if (outputs.mode == ShooterIOOutputMode.STOP) {
            topLeftMotor.stopMotor();
            bottomLeftMotor.stopMotor();
            topRightMotor.stopMotor();
            bottomRightMotor.stopMotor();

            // reapply followers
            bottomLeftMotor.setControl(shooterFollowerLeft);
            bottomRightMotor.setControl(shooterFollowerRight);
        } else if (outputs.mode == ShooterIOOutputMode.VELOCITY_TORQUE_CURRENT_FOC) {
            shooterVelocityControl.withVelocity(outputs.targetVelocity);
            topLeftMotor.setControl(shooterVelocityControl);
            topRightMotor.setControl(shooterVelocityControl);
        }
    }
}
