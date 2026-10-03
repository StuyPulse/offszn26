package com.stuypulse.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;

import com.stuypulse.robot.constants.GlobalSettings;
import com.stuypulse.robot.subsystems.intake.IntakeConstants.*;
import com.stuypulse.robot.subsystems.intake.IntakeIO.IntakeIOOutputs;
import com.stuypulse.robot.subsystems.intake.IntakeIO.PivotIOOutputMode;
import com.stuypulse.robot.subsystems.intake.IntakeIO.RollerIOOutputMode;
import com.stuypulse.robot.util.FullSubsystem;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends FullSubsystem {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs;
    private final IntakeIOOutputs outputs;

    @AutoLogOutput(key = "Intake/Pivot/State")
    private PivotState pivotState;

    @AutoLogOutput(key = "Intake/Rollers/State")
    private RollerState rollerState;

    private boolean hasHomed;

    public Intake(IntakeIO io) {
        this.io = io;
        inputs = new IntakeIOInputsAutoLogged();
        outputs = new IntakeIOOutputs();

        hasHomed = false;

        setPivotState(PivotState.STOW);
        setRollerState(RollerState.STOP);
    }

    public enum PivotState {
        DEPLOY,
        HOME,
        STOW
    }

    public enum RollerState {
        INTAKE,
        OUTTAKE,
        STOP
    }

    private void setPivotState(PivotState state) {
        this.pivotState = state;
    }

    private void setRollerState(RollerState state) {
        this.rollerState = state;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        if (!GlobalSettings.EnabledSubsystems.INTAKE.get()) {
            stopPivotMotor();
            stopRollerMotors();

            return;
        }

        if (!hasHomed) {
            setPivotState(PivotState.HOME);
        }

        switch (pivotState) {
            case HOME -> {
                if (pivotStalling()) {
                    io.seedPivotPosition(IntakeSettings.PIVOT_DEPLOY_ANGLE);
                    setPivotState(PivotState.DEPLOY);
                    hasHomed = true;
                } else {
                    runPivotVoltage(IntakeSettings.PIVOT_HOMING_VOLTAGE);
                }
            }

            case DEPLOY -> runPivotPosition(IntakeSettings.PIVOT_DEPLOY_ANGLE);

            case STOW -> runPivotPosition(IntakeSettings.PIVOT_STOW_ANGLE);
        }

        if (!canRunRollers()) {
            stopRollerMotors();

            return;
        }

        switch (rollerState) {
            case INTAKE -> runRollersDutyCycle(IntakeSettings.INTAKE_DUTY_CYCLE);

            case OUTTAKE -> runRollersDutyCycle(IntakeSettings.OUTTAKE_DUTY_CYCLE);

            case STOP -> stopRollerMotors();
        }
    }

    @Override
    public void periodicAfterScheduler() {
        io.applyOutputs(outputs);
    }

    private boolean pivotStalling() {
        return inputs.pivotInputs.statorCurrent.abs(Amps)
                > IntakeSettings.PIVOT_STALL_CURRENT.in(Amps);
    }

    private boolean canRunRollers() {
        return inputs.pivotInputs.position.lte(IntakeSettings.ROLLER_START_THRESHOLD)
                && pivotState == PivotState.DEPLOY;
    }

    private void runPivotPosition(Angle position) {
        outputs.pivotMode = PivotIOOutputMode.POSITION;
        outputs.pivotTargetPosition = position;
    }

    private void runPivotVoltage(Voltage voltage) {
        outputs.pivotMode = PivotIOOutputMode.VOLTAGE;
        outputs.pivotTargetVoltage = voltage;
    }

    private void runRollersDutyCycle(double dutyCycle) {
        outputs.rollerMode = RollerIOOutputMode.DUTY_CYCLE;
        outputs.rollerTargetDutyCycle = dutyCycle;
    }

    private void stopPivotMotor() {
        outputs.pivotMode = PivotIOOutputMode.STOP;
    }

    private void stopRollerMotors() {
        outputs.rollerMode = RollerIOOutputMode.STOP;
    }

    public Command intake() {
        return runOnce(
                        () -> {
                            setPivotState(PivotState.DEPLOY);
                            setRollerState(RollerState.INTAKE);
                        })
                .withName("Intake Intake");
    }

    public Command outtake() {
        return runOnce(
                        () -> {
                            setPivotState(PivotState.DEPLOY);
                            setRollerState(RollerState.OUTTAKE);
                        })
                .withName("Intake Outtake");
    }

    public Command stow() {
        return runOnce(
                        () -> {
                            setPivotState(PivotState.STOW);
                            setRollerState(RollerState.STOP);
                        })
                .withName("Intake Stow");
    }

    public Command stopRollers() {
        return runOnce(
                        () -> {
                            setPivotState(PivotState.DEPLOY);
                            setRollerState(RollerState.STOP);
                        })
                .withName("Intake Stop Rollers");
    }
}
