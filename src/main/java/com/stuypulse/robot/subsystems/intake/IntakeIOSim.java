package com.stuypulse.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.constants.GlobalSettings;
import com.stuypulse.robot.subsystems.intake.IntakeConstants.*;
import com.stuypulse.robot.util.simulation.TalonFXSimulation.SystemSim;
import com.stuypulse.robot.util.simulation.TalonFXSimulation.TalonFXSimulation;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakeIOSim implements IntakeIO {
    private final SystemSim<SingleJointedArmSim> pivotSim;
    private final SystemSim<DCMotorSim> rollerSim;

    private final TalonFXSimulation pivotMotor;
    private final TalonFXSimulation rollerLeaderMotor;
    private final TalonFXSimulation rollerFollowerMotor;

    private final DutyCycleOut rollerLeaderController;
    private final Follower rollerFollowerController;
    private final PositionVoltage pivotPositionController;
    private final VoltageOut pivotVoltageController;

    public IntakeIOSim() {
        this.pivotSim =
                SystemSim.of(
                        new SingleJointedArmSim(
                                LinearSystemId.createDCMotorSystem(
                                        DCMotor.getKrakenX60Foc(1),
                                        IntakeSettings.PIVOT_MOI.in(KilogramSquareMeters),
                                        IntakeSettings.PIVOT_GEAR_RATIO),
                                DCMotor.getKrakenX60Foc(1),
                                IntakeSettings.PIVOT_GEAR_RATIO,
                                IntakeSettings.ARM_LENGTH.in(Meters),
                                IntakeSettings.PIVOT_MIN_ANGLE.in(Radians),
                                IntakeSettings.PIVOT_MAX_ANGLE.in(Radians),
                                true,
                                IntakeSettings.PIVOT_MAX_ANGLE.in(Radians)));

        this.rollerSim =
                SystemSim.of(
                        new DCMotorSim(
                                LinearSystemId.createDCMotorSystem(
                                        DCMotor.getKrakenX60Foc(2),
                                        0.01, // arbitrary
                                        1.0),
                                DCMotor.getKrakenX60Foc(2)));

        this.pivotMotor =
                new TalonFXSimulation(
                        IntakeDeviceIds.PIVOT, IntakeSettings.PIVOT_GEAR_RATIO, pivotSim);
        this.rollerLeaderMotor =
                new TalonFXSimulation(IntakeDeviceIds.ROLLER_LEADER, 1.0, rollerSim);
        this.rollerFollowerMotor =
                new TalonFXSimulation(IntakeDeviceIds.ROLLER_FOLLOWER, 1.0, rollerSim);

        pivotMotor.configure(IntakeMotorConfigs.PIVOT_CONFIG);
        rollerLeaderMotor.configure(IntakeMotorConfigs.ROLLER_CONFIG);
        rollerFollowerMotor.configure(IntakeMotorConfigs.ROLLER_CONFIG);

        rollerFollowerMotor.linkToReference(rollerLeaderMotor);

        this.rollerLeaderController = new DutyCycleOut(0).withEnableFOC(true);
        this.rollerFollowerController =
                new Follower(rollerLeaderMotor.getDeviceID(), MotorAlignmentValue.Opposed);
        this.pivotPositionController = new PositionVoltage(0).withEnableFOC(true);
        this.pivotVoltageController = new VoltageOut(0).withEnableFOC(true);

        rollerFollowerMotor.setControl(rollerFollowerController);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        this.pivotSim.update(GlobalSettings.DT);
        this.pivotMotor.refresh();

        this.rollerSim.update(GlobalSettings.DT);
        this.rollerLeaderMotor.refresh();
        this.rollerFollowerMotor.refresh();

        pivotMotor.updateInputs(inputs.pivotInputs);
        rollerLeaderMotor.updateInputs(inputs.rollerLeftInputs);
        rollerFollowerMotor.updateInputs(inputs.rollerRightInputs);
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
            case DUTY_CYCLE -> rollerLeaderMotor.setControl(
                    rollerLeaderController.withOutput(outputs.rollerTargetDutyCycle));
            case STOP -> {
                rollerLeaderMotor.stopMotor();
                rollerFollowerMotor.stopMotor();
                rollerFollowerMotor.setControl(rollerFollowerController);
            }
        }
    }

    @Override
    public void seedPivotPosition(Angle position) {
        pivotMotor.setPosition(position);
    }
}
