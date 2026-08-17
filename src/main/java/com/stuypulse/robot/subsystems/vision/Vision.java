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

package com.stuypulse.robot.subsystems.vision;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.GlobalSettings;
import com.stuypulse.robot.subsystems.vision.VisionConstants.*;
import com.stuypulse.robot.subsystems.vision.VisionIO.MegaTagMode;
import com.stuypulse.robot.subsystems.vision.VisionIO.PoseObservation;
import com.stuypulse.robot.subsystems.vision.VisionIO.PoseObservationType;
import com.stuypulse.robot.subsystems.vision.VisionIO.VisionIOOutputs;
import com.stuypulse.robot.util.FullSubsystem;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Vision extends FullSubsystem {
    private final VisionConsumer consumer;
    private final VisionIO[] io;
    private final VisionIOInputsAutoLogged[] inputs;
    private final VisionIOOutputs[] outputs;
    private final Alert[] disconnectedAlerts;
    private int maxTagCount;

    private final Debouncer hasDataDebouncer;
    private boolean hasData;

    public Vision(VisionConsumer consumer, VisionIO... io) {
        this.consumer = consumer;
        this.io = io;

        // Initialize inputs
        this.inputs = new VisionIOInputsAutoLogged[io.length];
        this.outputs = new VisionIOOutputs[io.length];
        for (int i = 0; i < inputs.length; i++) {
            inputs[i] = new VisionIOInputsAutoLogged();
            outputs[i] = new VisionIOOutputs();
        }

        // Initialize disconnected alerts
        this.disconnectedAlerts = new Alert[io.length];
        for (int i = 0; i < inputs.length; i++) {
            disconnectedAlerts[i] =
                    new Alert(
                            "Vision camera " + CamerasList.CAMERAS[i].name() + " is disconnected.",
                            AlertType.kWarning);
        }

        maxTagCount = 0;

        hasDataDebouncer =
                new Debouncer(VisionConstants.VisionSettings.BUZZ_DEBOUNCE, DebounceType.kBoth);
    }

    /**
     * Returns the X angle to the best target, which can be used for simple servoing with vision.
     *
     * @param cameraIndex The index of the camera to use.
     */
    public Rotation2d getTargetX(int cameraIndex) {
        return inputs[cameraIndex].latestTargetObservation.tx();
    }

    public int getMaxTagCount() {
        return maxTagCount;
    }

    public boolean isCameraDead(Camera camera) {
        return !inputs[camera.ordinal()].connected;
    }

    public boolean hasData() {
        return hasDataDebouncer.calculate(hasData);
    }

    @Override
    public void periodic() {
        maxTagCount = 0;
        hasData = false;

        for (int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            Logger.processInputs("Vision/" + CamerasList.CAMERAS[i].name(), inputs[i]);
        }

        if (!GlobalSettings.EnabledSubsystems.VISION.get()) {
            return;
        }

        // Initialize logging values
        List<Pose3d> allTagPoses = new LinkedList<>();
        List<Pose3d> allRobotPoses = new LinkedList<>();
        List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
        List<Pose3d> allRobotPosesRejected = new LinkedList<>();
        List<Pose3d> allObjectPoses = new ArrayList<>();

        // Loop over cameras
        for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
            // Update disconnected alert
            disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

            // Initialize logging values
            List<Pose3d> tagPoses = new LinkedList<>();
            List<Pose3d> robotPoses = new LinkedList<>();
            List<Pose3d> robotPosesAccepted = new LinkedList<>();
            List<Pose3d> robotPosesRejected = new LinkedList<>();

            // Add tag poses
            for (int tagId : inputs[cameraIndex].tagIds) {
                var tagPose = Field.APRIL_TAG_LAYOUT.getTagPose(tagId);
                if (tagPose.isPresent()) {
                    tagPoses.add(tagPose.get());
                }
            }

            // add every object pose to the main list
            Collections.addAll(allObjectPoses, inputs[cameraIndex].objectPoses);

            // Loop over pose observations
            for (PoseObservation observation : inputs[cameraIndex].poseObservations) {
                maxTagCount = Math.max(maxTagCount, observation.tagCount());
                // Check whether to reject pose
                boolean rejectPose =
                        observation.tagCount() == 0 // Must have at least one tag
                                || (observation.tagCount() == 1
                                        && observation.ambiguity()
                                                > VisionSettings
                                                        .MAX_AMBIGUITY) // Cannot be high ambiguity
                                || Math.abs(observation.pose().getZ())
                                        > VisionSettings.MAX_Z_ERROR // Must have realistic Z
                                // coordinate

                                // Must be within the field boundaries
                                || observation.pose().getX() < 0.0
                                || observation.pose().getX()
                                        > Field.APRIL_TAG_LAYOUT.getFieldLength()
                                || observation.pose().getY() < 0.0
                                || observation.pose().getY()
                                        > Field.APRIL_TAG_LAYOUT.getFieldWidth();

                // Add pose to log
                robotPoses.add(observation.pose());
                if (rejectPose) {
                    robotPosesRejected.add(observation.pose());
                } else {
                    robotPosesAccepted.add(observation.pose());
                }

                // Skip if rejected
                if (rejectPose) {
                    continue;
                }

                hasData = true;

                // Calculate standard deviations
                double stdDevFactor =
                        Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
                double linearStdDev = VisionSettings.LINEAR_STD_DEV_BASELINE * stdDevFactor;
                double angularStdDev = VisionSettings.ANGULAR_STD_DEV_BASELINE * stdDevFactor;
                if (observation.type() == PoseObservationType.MEGATAG_2) {
                    linearStdDev *= VisionSettings.LINEAR_STD_DEV_MEGATAG_2_FACTOR;
                    angularStdDev *= VisionSettings.ANGULAR_STD_DEV_MEGATAG_2_FACTOR;
                }
                linearStdDev *= CamerasList.CAMERAS[cameraIndex].stdDevFactor();
                angularStdDev *= CamerasList.CAMERAS[cameraIndex].stdDevFactor();

                // Send vision observation
                consumer.accept(
                        observation.pose().toPose2d(),
                        observation.timestamp(),
                        VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
            }

            // Log camera datadata
            Logger.recordOutput(
                    "Vision/Camera" + CamerasList.CAMERAS[cameraIndex].name() + "/TagPoses",
                    tagPoses.toArray(new Pose3d[tagPoses.size()]));
            Logger.recordOutput(
                    "Vision/Camera" + CamerasList.CAMERAS[cameraIndex].name() + "/RobotPoses",
                    robotPoses.toArray(new Pose3d[robotPoses.size()]));
            Logger.recordOutput(
                    "Vision/Camera"
                            + CamerasList.CAMERAS[cameraIndex].name()
                            + "/RobotPosesAccepted",
                    robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
            Logger.recordOutput(
                    "Vision/Camera"
                            + CamerasList.CAMERAS[cameraIndex].name()
                            + "/RobotPosesRejected",
                    robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
            allTagPoses.addAll(tagPoses);
            allRobotPoses.addAll(robotPoses);
            allRobotPosesAccepted.addAll(robotPosesAccepted);
            allRobotPosesRejected.addAll(robotPosesRejected);
        }

        // Log summary data
        Logger.recordOutput(
                "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
        Logger.recordOutput(
                "Vision/Summary/RobotPoses",
                allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
        Logger.recordOutput(
                "Vision/Summary/RobotPosesAccepted",
                allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
        Logger.recordOutput(
                "Vision/Summary/RobotPosesRejected",
                allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
        Logger.recordOutput(
                "Vision/Summary/ObjectPoses",
                allObjectPoses.toArray(new Pose3d[allObjectPoses.size()]));
    }

    @Override
    public void periodicAfterScheduler() {
        for (int i = 0; i < io.length; i++) {
            Logger.recordOutput(
                    "Vision/" + CamerasList.CAMERAS[i].name() + "/MegaTagMode",
                    outputs[i].megaTagMode);
            Logger.recordOutput(
                    "Vision/" + CamerasList.CAMERAS[i].name() + "/Pipeline", outputs[i].pipeline);

            io[i].applyOutputs(outputs[i]);
        }
    }

    @FunctionalInterface
    public interface VisionConsumer {
        void accept(
                Pose2d visionRobotPoseMeters,
                double timestampSeconds,
                Matrix<N3, N1> visionMeasurementStdDevs);
    }

    public Command setMegaTagMode(MegaTagMode mode) {
        return runOnce(
                        () -> {
                            for (VisionIOOutputs output : outputs) {
                                output.megaTagMode = mode;
                            }
                        })
                .withName("Vision Set MegaTag Mode")
                .ignoringDisable(true);
    }

    public Command setPipeline(int pipeline) {
        return runOnce(
                        () -> {
                            for (VisionIOOutputs output : outputs) {
                                output.pipeline = pipeline;
                            }
                        })
                .withName("Vision Set Pipeline")
                .ignoringDisable(true);
    }

    public Command setAprilTagWhitelist(int[] whitelist) {
        return runOnce(
                        () -> {
                            for (VisionIOOutputs output : outputs) {
                                output.aprilTagIDWhitelist = whitelist;
                            }
                        })
                .withName("Vision Set AprilTag Whitelist")
                .ignoringDisable(true);
    }

    public Command setIMUMode(int imuMode) {
        return runOnce(
                        () -> {
                            for (VisionIOOutputs output : outputs) {
                                output.imuMode = imuMode;
                            }
                        })
                .withName("Vision Set IMU Mode")
                .ignoringDisable(true);
    }

    public Command resetIMU() {
        return setIMUMode(VisionSettings.RESET_IMU_INDEX).withName("Vision Reset IMU");
    }
}
