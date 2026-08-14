package com.stuypulse.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.constants.GlobalSettings;
import com.stuypulse.robot.subsystems.intake.IntakeConstants.*;
import com.stuypulse.robot.util.simulation.TalonFXSimulation.SystemSim;
import com.stuypulse.robot.util.simulation.TalonFXSimulation.TalonFXSimulation;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.*;
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
  private final TorqueCurrentFOC pivotPushdownController;

  private final StatusSignal<Angle> pivotPosition;
  private final StatusSignal<Current> pivotSupplyCurrent;
  private final StatusSignal<Current> pivotStatorCurrent;
  private final StatusSignal<Temperature> pivotTemperature;
  private final StatusSignal<Voltage> pivotAppliedVoltage;
  private final StatusSignal<AngularVelocity> pivotVelocity;

  private final StatusSignal<Angle> rollerLeaderPosition;
  private final StatusSignal<Current> rollerLeaderSupplyCurrent;
  private final StatusSignal<Current> rollerLeaderStatorCurrent;
  private final StatusSignal<Temperature> rollerLeaderTemperature;
  private final StatusSignal<Voltage> rollerLeaderAppliedVoltage;
  private final StatusSignal<AngularVelocity> rollerLeaderVelocity;

  private final StatusSignal<Angle> rollerFollowerPosition;
  private final StatusSignal<Current> rollerFollowerSupplyCurrent;
  private final StatusSignal<Current> rollerFollowerStatorCurrent;
  private final StatusSignal<Temperature> rollerFollowerTemperature;
  private final StatusSignal<Voltage> rollerFollowerAppliedVoltage;
  private final StatusSignal<AngularVelocity> rollerFollowerVelocity;

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
        new TalonFXSimulation(IntakeDeviceIds.PIVOT, IntakeSettings.PIVOT_GEAR_RATIO, pivotSim);
    this.rollerLeaderMotor = new TalonFXSimulation(IntakeDeviceIds.ROLLER_LEADER, 1.0, rollerSim);
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
    this.pivotPushdownController = new TorqueCurrentFOC(0);

    rollerFollowerMotor.setControl(rollerFollowerController);

    this.pivotPosition = pivotMotor.getPosition();
    this.pivotSupplyCurrent = pivotMotor.getSupplyCurrent();
    this.pivotStatorCurrent = pivotMotor.getStatorCurrent();
    this.pivotTemperature = pivotMotor.getDeviceTemp();
    this.pivotAppliedVoltage = pivotMotor.getMotorVoltage();
    this.pivotVelocity = pivotMotor.getVelocity();

    this.rollerLeaderPosition = rollerLeaderMotor.getPosition();
    this.rollerLeaderSupplyCurrent = rollerLeaderMotor.getSupplyCurrent();
    this.rollerLeaderStatorCurrent = rollerLeaderMotor.getStatorCurrent();
    this.rollerLeaderTemperature = rollerLeaderMotor.getDeviceTemp();
    this.rollerLeaderAppliedVoltage = rollerLeaderMotor.getMotorVoltage();
    this.rollerLeaderVelocity = rollerLeaderMotor.getVelocity();

    this.rollerFollowerPosition = rollerFollowerMotor.getPosition();
    this.rollerFollowerSupplyCurrent = rollerFollowerMotor.getSupplyCurrent();
    this.rollerFollowerStatorCurrent = rollerFollowerMotor.getStatorCurrent();
    this.rollerFollowerTemperature = rollerFollowerMotor.getDeviceTemp();
    this.rollerFollowerAppliedVoltage = rollerFollowerMotor.getMotorVoltage();
    this.rollerFollowerVelocity = rollerFollowerMotor.getVelocity();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    this.pivotSim.update(GlobalSettings.DT);
    this.pivotMotor.refresh();

    this.rollerSim.update(GlobalSettings.DT);
    this.rollerLeaderMotor.refresh();
    this.rollerFollowerMotor.refresh();

    BaseStatusSignal.refreshAll(
        pivotPosition,
        pivotSupplyCurrent,
        pivotStatorCurrent,
        pivotTemperature,
        pivotAppliedVoltage,
        pivotVelocity,
        rollerLeaderPosition,
        rollerLeaderSupplyCurrent,
        rollerLeaderStatorCurrent,
        rollerLeaderTemperature,
        rollerLeaderAppliedVoltage,
        rollerLeaderVelocity,
        rollerFollowerPosition,
        rollerFollowerSupplyCurrent,
        rollerFollowerStatorCurrent,
        rollerFollowerTemperature,
        rollerFollowerAppliedVoltage,
        rollerFollowerVelocity);

    inputs.pivotMotorPosition = pivotPosition.getValue();
    inputs.pivotMotorSupplyCurrent = pivotSupplyCurrent.getValue();
    inputs.pivotMotorStatorCurrent = pivotStatorCurrent.getValue();
    inputs.pivotMotorTemperature = pivotTemperature.getValue();
    inputs.pivotMotorAppliedVoltage = pivotAppliedVoltage.getValue();
    inputs.pivotMotorVelocity = pivotVelocity.getValue();

    inputs.rollerLeaderMotorPosition = rollerLeaderPosition.getValue();
    inputs.rollerLeaderMotorSupplyCurrent = rollerLeaderSupplyCurrent.getValue();
    inputs.rollerLeaderMotorStatorCurrent = rollerLeaderStatorCurrent.getValue();
    inputs.rollerLeaderMotorTemperature = rollerLeaderTemperature.getValue();
    inputs.rollerLeaderMotorAppliedVoltage = rollerLeaderAppliedVoltage.getValue();
    inputs.rollerLeaderMotorVelocity = rollerLeaderVelocity.getValue();

    inputs.rollerFollowerMotorPosition = rollerFollowerPosition.getValue();
    inputs.rollerFollowerMotorSupplyCurrent = rollerFollowerSupplyCurrent.getValue();
    inputs.rollerFollowerMotorStatorCurrent = rollerFollowerStatorCurrent.getValue();
    inputs.rollerFollowerMotorTemperature = rollerFollowerTemperature.getValue();
    inputs.rollerFollowerMotorAppliedVoltage = rollerFollowerAppliedVoltage.getValue();
    inputs.rollerFollowerMotorVelocity = rollerFollowerVelocity.getValue();
  }

  @Override
  public void applyOutputs(IntakeIOOutputs outputs) {
    switch (outputs.pivotMode) {
      case POSITION -> pivotMotor.setControl(
          pivotPositionController
              .withPosition(outputs.pivotTargetPosition)
              .withSlot(outputs.pivotGainSlot));
      case TORQUE_CURRENT -> pivotMotor.setControl(
          pivotPushdownController.withOutput(outputs.pivotTargetTorqueCurrent));
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
}
