package com.stuypulse.robot.subsystems.hood;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.stuypulse.robot.constants.GlobalSettings;
import com.stuypulse.robot.subsystems.hood.HoodConstants.*;
import com.stuypulse.robot.util.simulation.TalonFXSimulation.SystemSim;
import com.stuypulse.robot.util.simulation.TalonFXSimulation.TalonFXSimulation;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class HoodIOSim implements HoodIO {

    private final SystemSim<DCMotorSim> hoodSystem;

    private final TalonFXSimulation hoodMotor;

    private final PositionVoltage positionController;

    public HoodIOSim() {
        hoodSystem =
                SystemSim.of(
                        new DCMotorSim(
                                LinearSystemId.createDCMotorSystem(
                                        DCMotor.getKrakenX44Foc(1), 0.01, 1),
                                DCMotor.getKrakenX44Foc(1)));

        hoodMotor = new TalonFXSimulation(HoodDeviceIds.HOOD_MOTOR, 1, hoodSystem);

        hoodMotor.configure(HoodMotorConfigs.HOOD_CONFIG);

        positionController = new PositionVoltage(0).withEnableFOC(true);
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        hoodSystem.update(GlobalSettings.DT);
        hoodMotor.refresh();

        hoodMotor.updateInputs(inputs.hoodInputs);
    }

    @Override
    public void applyOutputs(HoodIOOutputs outputs) {
        switch (outputs.hoodMode) {
            case POSITION -> hoodMotor.setControl(
                    positionController.withPosition(outputs.hoodTargetPosition));

            case STOP -> hoodMotor.stopMotor();
        }
    }
}
