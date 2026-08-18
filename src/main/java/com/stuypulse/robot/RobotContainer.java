/************************ PROJECT PHIL ************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/
package com.stuypulse.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.stuypulse.robot.commands.DriveCommands;
import com.stuypulse.robot.constants.DriverConstants;
import com.stuypulse.robot.constants.GlobalSettings;
import com.stuypulse.robot.constants.GlobalSettings.VisionMode;
import com.stuypulse.robot.subsystems.swerve.GyroIO;
import com.stuypulse.robot.subsystems.swerve.GyroIOReal;
import com.stuypulse.robot.subsystems.swerve.ModuleIO;
import com.stuypulse.robot.subsystems.swerve.ModuleIOReal;
import com.stuypulse.robot.subsystems.swerve.ModuleIOSim;
import com.stuypulse.robot.subsystems.swerve.Swerve;
import com.stuypulse.robot.subsystems.swerve.TunerConstants;
import com.stuypulse.robot.subsystems.vision.Vision;
import com.stuypulse.robot.subsystems.vision.VisionConstants.CamerasList;
import com.stuypulse.robot.subsystems.vision.VisionIO;
import com.stuypulse.robot.subsystems.vision.VisionIOLimelight;
import com.stuypulse.robot.subsystems.vision.VisionIOPhotonVision;
import com.stuypulse.robot.subsystems.vision.VisionIOPhotonVisionSim;
import com.stuypulse.robot.util.InterpolationCalculator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.Arrays;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Swerve swerve;
  private final Vision vision;

  // Controller
  private final CommandXboxController controller;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {

    switch (GlobalSettings.CURRENT_MODE) {
      case REAL -> {
        swerve =
            new Swerve(
                new GyroIOReal(),
                new ModuleIOReal(TunerConstants.FrontLeft),
                new ModuleIOReal(TunerConstants.FrontRight),
                new ModuleIOReal(TunerConstants.BackLeft),
                new ModuleIOReal(TunerConstants.BackRight));

        if (GlobalSettings.VISION_MODE == VisionMode.LIMELIGHT_VISION) {
          vision =
              new Vision(
                  swerve,
                  Arrays.stream(CamerasList.CAMERAS)
                      .map((camera) -> new VisionIOLimelight(camera.name(), swerve::getRotation))
                      .toArray(VisionIO[]::new));
        } else {
          vision =
              new Vision(
                  swerve,
                  Arrays.stream(CamerasList.CAMERAS)
                      .map(
                          (camera) ->
                              new VisionIOPhotonVision(camera.name(), camera.robotToCamera()))
                      .toArray(VisionIO[]::new));
        }
      }

      case SIM -> {
        swerve =
            new Swerve(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        vision =
            new Vision(
                swerve,
                Arrays.stream(CamerasList.CAMERAS)
                    .map(
                        (camera) ->
                            new VisionIOPhotonVisionSim(
                                camera.name(), camera.robotToCamera(), swerve::getPose))
                    .toArray(VisionIO[]::new));
      }

        // For replay mode
      default -> {
        swerve =
            new Swerve(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        vision =
            new Vision(
                swerve,
                Arrays.stream(CamerasList.CAMERAS)
                    .map((camera) -> new VisionIO() {})
                    .toArray(VisionIO[]::new));
      }
    }

    this.controller = new CommandXboxController(DriverConstants.Driver.DRIVER_INDEX);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    configureButtonBindings();
    configureDefaultCommands();
    configureAutons();

    // COMMENT OUT THIS METHOD BEFORE RUNNING MATCHES
    configureSysid();
  }

  private void configureDefaultCommands() {
    swerve.setDefaultCommand(DriveCommands.joystickDrive(swerve, controller));
  }

  private void configureAutons() {}

  private void configureSysid() {
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(swerve));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(swerve));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        swerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        swerve.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", swerve.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", swerve.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    controller.a().onTrue(DriveCommands.alignToHub(swerve));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void clearMemoized() {
    InterpolationCalculator.clearMemoized();
  }
}
