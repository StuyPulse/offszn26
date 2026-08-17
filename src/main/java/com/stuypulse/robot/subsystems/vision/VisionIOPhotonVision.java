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
import com.stuypulse.robot.subsystems.vision.VisionConstants.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhotonVision implements VisionIO {
    protected final PhotonCamera camera;
    protected final Transform3d robotToCamera;
    protected final Supplier<Pose2d> poseSupplier;

    /**
     * Creates a new VisionIOPhotonVision.
     *
     * @param name The configured name of the camera.
     * @param rotationSupplier The 3D position of the camera relative to the robot.
     */
    public VisionIOPhotonVision(
            String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
        camera = new PhotonCamera(name);
        this.robotToCamera = robotToCamera;
        this.poseSupplier = poseSupplier;
    }

    private Optional<Pose3d> getFuelFieldPose(
            Pose3d currentRobotPose, Transform3d robotToCamera, PhotonTrackedTarget target) {

        final double yawRad = Units.degreesToRadians(-target.getYaw()); // negative
        final double pitchRad = Units.degreesToRadians(target.getPitch());

        SmartDashboard.putNumber("Vision/Fuel Yaw", Units.radiansToDegrees(yawRad));
        SmartDashboard.putNumber("Vision/Fuel Pitch", Units.radiansToDegrees(pitchRad));

        final double cameraPitchRad = -robotToCamera.getRotation().getY(); // negative

        final double distance =
                PhotonUtils.calculateDistanceToTargetMeters(
                        robotToCamera.getZ(), VisionSettings.FUEL_RADIUS, cameraPitchRad, pitchRad);

        if (distance <= 0 || Double.isNaN(distance)) {
            SmartDashboard.putNumber(
                    "Vision/target_" + target.getDetectedObjectClassID() + "distance_m", distance);
            return Optional.empty();
        }

        final Translation3d camToPiece =
                new Translation3d(
                        PhotonUtils.estimateCameraToTargetTranslation(
                                distance, new Rotation2d(yawRad)));

        final Pose3d fieldToCamera = currentRobotPose.transformBy(robotToCamera);
        final Pose3d fieldToPiece =
                fieldToCamera.transformBy(new Transform3d(camToPiece, new Rotation3d()));

        return Optional.of(fieldToPiece);
    }

    private void updateObjectDetectionInputs(VisionIOInputs inputs) {
        final List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        List<Pose3d> objectPoses = new ArrayList<>();
        // DogLog.log("Vision/" + currentCamera.getName() + "_numResults/",
        // results.size());
        for (PhotonPipelineResult result : results) {
            if (!result.hasTargets()) {
                continue;
            }

            Optional<Pose3d> fuelPose =
                    this.getFuelFieldPose(
                            new Pose3d(poseSupplier.get()),
                            this.robotToCamera,
                            result.getBestTarget());
            if (fuelPose.isPresent()) {
                // DogLog.log("Vision/" + currentCamera.getName() + "_FuelPose",
                // fuelPose.get());
                objectPoses.add(fuelPose.get());
            }
        }
        inputs.objectPoses = new Pose3d[objectPoses.size()];
        for (int i = 0; i < objectPoses.size(); i++) {
            inputs.objectPoses[i] = objectPoses.get(i);
        }
    }

    private void updateAprilTagInputs(VisionIOInputs inputs) {
        // Read new camera observations
        Set<Short> tagIds = new HashSet<>();
        List<PoseObservation> poseObservations = new LinkedList<>();
        for (var result : camera.getAllUnreadResults()) {
            // Update latest target observation
            if (result.hasTargets()) {
                inputs.latestTargetObservation =
                        new TargetObservation(
                                Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                                Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
            } else {
                inputs.latestTargetObservation =
                        new TargetObservation(new Rotation2d(), new Rotation2d());
            }

            // Add pose observation
            if (result.multitagResult.isPresent()) { // Multitag result
                var multitagResult = result.multitagResult.get();

                // Calculate robot pose
                Transform3d fieldToCamera = multitagResult.estimatedPose.best;
                Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
                Pose3d robotPose =
                        new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                // Calculate average tag distance
                double totalTagDistance = 0.0;
                for (var target : result.targets) {
                    totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
                }

                // Add tag IDs
                tagIds.addAll(multitagResult.fiducialIDsUsed);

                // Add observation
                poseObservations.add(
                        new PoseObservation(
                                result.getTimestampSeconds(), // Timestamp
                                robotPose, // 3D pose estimate
                                multitagResult.estimatedPose.ambiguity, // Ambiguity
                                multitagResult.fiducialIDsUsed.size(), // Tag count
                                totalTagDistance / result.targets.size(), // Average tag distance
                                PoseObservationType.PHOTONVISION)); // Observation type

            } else if (!result.targets.isEmpty()) { // Single tag result
                var target = result.targets.get(0);

                // Calculate robot pose
                var tagPose = Field.APRIL_TAG_LAYOUT.getTagPose(target.fiducialId);
                if (tagPose.isPresent()) {
                    Transform3d fieldToTarget =
                            new Transform3d(
                                    tagPose.get().getTranslation(), tagPose.get().getRotation());
                    Transform3d cameraToTarget = target.bestCameraToTarget;
                    Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
                    Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
                    Pose3d robotPose =
                            new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                    // Add tag ID
                    tagIds.add((short) target.fiducialId);

                    // Add observation
                    poseObservations.add(
                            new PoseObservation(
                                    result.getTimestampSeconds(), // Timestamp
                                    robotPose, // 3D pose estimate
                                    target.poseAmbiguity, // Ambiguity
                                    1, // Tag count
                                    cameraToTarget
                                            .getTranslation()
                                            .getNorm(), // Average tag distance
                                    PoseObservationType.PHOTONVISION)); // Observation type
                }
            }
        }

        // Save pose observations to inputs object
        inputs.poseObservations = new PoseObservation[poseObservations.size()];
        for (int i = 0; i < poseObservations.size(); i++) {
            inputs.poseObservations[i] = poseObservations.get(i);
        }

        // Save tag IDs to inputs objects
        inputs.tagIds = new int[tagIds.size()];
        int i = 0;
        for (int id : tagIds) {
            inputs.tagIds[i++] = id;
        }
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.connected = camera.isConnected();

        switch (camera.getPipelineIndex()) {
            case 0 -> this.updateObjectDetectionInputs(inputs);
            case 1 -> this.updateAprilTagInputs(inputs);
        }
    }

    @Override
    public void applyOutputs(VisionIOOutputs outputs) {
        camera.setPipelineIndex(outputs.pipeline);
    }
}
