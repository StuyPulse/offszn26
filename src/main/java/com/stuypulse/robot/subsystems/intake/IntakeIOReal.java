package com.stuypulse.robot.subsystems.intake;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.constants.GlobalSettings;
import com.stuypulse.robot.subsystems.intake.IntakeConstants.*;
import com.stuypulse.robot.util.logged.LoggedTalonFX.LoggedTalonFX;
import edu.wpi.first.units.measure.Angle;

public class IntakeIOReal implements IntakeIO {
    private final LoggedTalonFX pivotMotor;
    private final LoggedTalonFX rollerLeftMotor;
    private final LoggedTalonFX rollerRightMotor;

    private final DutyCycleOut rollerLeaderController;
    private final Follower rollerFollowerController;
    private final PositionVoltage pivotPositionController;
    private final VoltageOut pivotVoltageController;

    public IntakeIOReal() {
        this.pivotMotor = new LoggedTalonFX(IntakeDeviceIds.PIVOT, GlobalSettings.RIO);
        this.rollerLeftMotor = new LoggedTalonFX(IntakeDeviceIds.ROLLER_LEADER, GlobalSettings.RIO);
        this.rollerRightMotor =
                new LoggedTalonFX(IntakeDeviceIds.ROLLER_FOLLOWER, GlobalSettings.RIO);

        IntakeMotorConfigs.PIVOT_CONFIG.configure(pivotMotor);
        IntakeMotorConfigs.ROLLER_CONFIG.configure(rollerLeftMotor);
        IntakeMotorConfigs.ROLLER_CONFIG.configure(rollerRightMotor);

        this.rollerLeaderController = new DutyCycleOut(0).withEnableFOC(true);
        this.rollerFollowerController =
                new Follower(rollerLeftMotor.getDeviceID(), MotorAlignmentValue.Opposed);
        this.pivotPositionController = new PositionVoltage(0).withEnableFOC(true);
        this.pivotVoltageController = new VoltageOut(0).withEnableFOC(true);

        rollerRightMotor.setControl(rollerFollowerController);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        pivotMotor.updateInputs(inputs.pivotInputs);
        rollerLeftMotor.updateInputs(inputs.rollerLeftInputs);
        rollerRightMotor.updateInputs(inputs.rollerRightInputs);
    }

    @Override
    public void applyOutputs(IntakeIOOutputs outputs) {
        switch (outputs.pivotMode) {
            case POSITION -> pivotMotor.setControl(
                    pivotPositionController.withPosition(outputs.pivotTargetPosition));
            case VOLTAGE -> pivotMotor.setControl(
                    pivotVoltageController.withOutput(outputs.pivotTargetVoltage));
            case STOP -> pivotMotor.stopMotor();
        }

        switch (outputs.rollerMode) {
            case DUTY_CYCLE -> rollerLeftMotor.setControl(
                    rollerLeaderController.withOutput(outputs.rollerTargetDutyCycle));
            case STOP -> {
                rollerLeftMotor.stopMotor();
                rollerRightMotor.stopMotor();
                rollerRightMotor.setControl(rollerFollowerController);
            }
        }
    }

    @Override
    public void seedPivotPosition(Angle position) {
        pivotMotor.setPosition(position);
    }
}
