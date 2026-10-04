package com.stuypulse.robot.subsystems.feeder;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.stuypulse.robot.constants.GlobalSettings;
import com.stuypulse.robot.subsystems.feeder.FeederConstants.*;
import com.stuypulse.robot.util.logged.LoggedTalonFX.LoggedTalonFX;

public class FeederIOReal implements FeederIO {
    private final LoggedTalonFX feederLeader;
    private final LoggedTalonFX feederFollower;

    private final DutyCycleOut feederLeaderController;
    private final Follower feederFollowerController;

    public FeederIOReal() {
        feederLeader = new LoggedTalonFX(FeederDeviceIds.LEADER, GlobalSettings.RIO);
        feederFollower = new LoggedTalonFX(FeederDeviceIds.FOLLOWER, GlobalSettings.RIO);

        FeederMotorConfigs.FEEDER_CONFIG.configure(feederLeader);
        FeederMotorConfigs.FEEDER_CONFIG.configure(feederFollower);

        feederLeaderController = new DutyCycleOut(0).withEnableFOC(true);
        feederFollowerController =
                new Follower(feederLeader.getDeviceID(), MotorAlignmentValue.Opposed);

        feederFollower.setControl(feederFollowerController);
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        feederLeader.updateInputs(inputs.feederLeaderInputs);
        feederFollower.updateInputs(inputs.feederFollowerInputs);
    }

    @Override
    public void applyOutputs(FeederIOOutputs outputs) {
        switch (outputs.feederMode) {
            case DUTY_CYCLE -> feederLeader.setControl(
                    feederLeaderController.withOutput(outputs.targetDutyCycle));

            case STOP -> {
                feederLeader.stopMotor();
                feederFollower.stopMotor();

                feederFollower.setControl(feederFollowerController);
            }
        }
    }
}
