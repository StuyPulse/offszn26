package com.stuypulse.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.constants.GlobalSettings;
import com.stuypulse.robot.subsystems.shooter.ShooterConstants.*;
import com.stuypulse.robot.util.logged.LoggedTalonFX.LoggedTalonFX;

public final class ShooterIOReal implements ShooterIO {
    private final LoggedTalonFX topLeftMotor;
    private final LoggedTalonFX bottomLeftMotor;

    private final LoggedTalonFX topRightMotor;
    private final LoggedTalonFX bottomRightMotor;

    private final Follower shooterFollowerLeft;
    private final Follower shooterFollowerRight;

    private final VelocityTorqueCurrentFOC shooterVelocityControl;

    public ShooterIOReal() {
        this.topLeftMotor = new LoggedTalonFX(ShooterIds.TOP_LEFT_MOTOR, GlobalSettings.RIO);
        this.bottomLeftMotor = new LoggedTalonFX(ShooterIds.BOTTOM_LEFT_MOTOR, GlobalSettings.RIO);
        ShooterMotorConfigs.LEFT_CONFIG.configure(topLeftMotor);
        ShooterMotorConfigs.LEFT_CONFIG.configure(bottomLeftMotor);

        this.topRightMotor = new LoggedTalonFX(ShooterIds.TOP_RIGHT_MOTOR, GlobalSettings.RIO);
        this.bottomRightMotor =
                new LoggedTalonFX(ShooterIds.BOTTOM_RIGHT_MOTOR, GlobalSettings.RIO);
        ShooterMotorConfigs.RIGHT_CONFIG.configure(topRightMotor);
        ShooterMotorConfigs.RIGHT_CONFIG.configure(bottomRightMotor);

        this.shooterFollowerLeft =
                new Follower(topLeftMotor.getDeviceID(), MotorAlignmentValue.Aligned);
        this.shooterFollowerRight =
                new Follower(topRightMotor.getDeviceID(), MotorAlignmentValue.Opposed);

        this.shooterVelocityControl = new VelocityTorqueCurrentFOC(0);

        topLeftMotor.setControl(shooterVelocityControl);
        topRightMotor.setControl(shooterVelocityControl);
        bottomLeftMotor.setControl(shooterFollowerLeft);
        bottomRightMotor.setControl(shooterFollowerRight);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
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
