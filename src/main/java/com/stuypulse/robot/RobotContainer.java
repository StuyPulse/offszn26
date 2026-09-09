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
import com.stuypulse.robot.subsystems.feeder.Feeder;
import com.stuypulse.robot.subsystems.feeder.FeederIO;
import com.stuypulse.robot.subsystems.feeder.FeederIOReal;
import com.stuypulse.robot.subsystems.feeder.FeederIOSim;
import com.stuypulse.robot.subsystems.hood.Hood;
import com.stuypulse.robot.subsystems.hood.HoodIO;
import com.stuypulse.robot.subsystems.hood.HoodIOReal;
import com.stuypulse.robot.subsystems.hood.HoodIOSim;
import com.stuypulse.robot.subsystems.indexer.Indexer;
import com.stuypulse.robot.subsystems.indexer.IndexerIO;
import com.stuypulse.robot.subsystems.indexer.IndexerIOReal;
import com.stuypulse.robot.subsystems.indexer.IndexerIOSim;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.intake.IntakeIO;
import com.stuypulse.robot.subsystems.intake.IntakeIOReal;
import com.stuypulse.robot.subsystems.intake.IntakeIOSim;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.shooter.ShooterIO;
import com.stuypulse.robot.subsystems.shooter.ShooterIOReal;
import com.stuypulse.robot.subsystems.shooter.ShooterIOSim;
import com.stuypulse.robot.subsystems.swerve.GyroIO;
import com.stuypulse.robot.subsystems.swerve.GyroIOReal;
import com.stuypulse.robot.subsystems.swerve.ModuleIO;
import com.stuypulse.robot.subsystems.swerve.ModuleIOReal;
import com.stuypulse.robot.subsystems.swerve.ModuleIOSim;
import com.stuypulse.robot.subsystems.swerve.Swerve;
import com.stuypulse.robot.subsystems.swerve.TunerConstants;
import com.stuypulse.robot.subsystems.vision.Vision;
import com.stuypulse.robot.subsystems.vision.VisionConstants.Cameras;
import com.stuypulse.robot.subsystems.vision.VisionIO;
import com.stuypulse.robot.subsystems.vision.VisionIOLimelight;
import com.stuypulse.robot.subsystems.vision.VisionIOPhotonVision;
import com.stuypulse.robot.subsystems.vision.VisionIOPhotonVisionSim;
import com.stuypulse.robot.util.InterpolationCalculator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.EnumMap;
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
    private final Hood hood;
    private final Intake intake;
    private final Feeder feeder;
    private final Vision vision;
    private final Shooter shooter;
    private final Indexer indexer;

    // Controller
    private final CommandXboxController controller;

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, IO devices, and commands. */
    public RobotContainer() {

        EnumMap<Cameras, VisionIO> cameraIOMap = new EnumMap<>(Cameras.class);
        switch (GlobalSettings.CURRENT_MODE) {
            case REAL -> {
                swerve =
                        new Swerve(
                                new GyroIOReal(),
                                new ModuleIOReal(TunerConstants.FrontLeft),
                                new ModuleIOReal(TunerConstants.FrontRight),
                                new ModuleIOReal(TunerConstants.BackLeft),
                                new ModuleIOReal(TunerConstants.BackRight));

                shooter = new Shooter(new ShooterIOReal(), swerve::getPose);
                hood = new Hood(new HoodIOReal(), swerve::getPose);
                intake = new Intake(new IntakeIOReal());
                feeder = new Feeder(new FeederIOReal());
                indexer = new Indexer(new IndexerIOReal());

                for (Cameras camera : Cameras.values()) {
                    if (GlobalSettings.VISION_MODE == VisionMode.LIMELIGHT_VISION) {
                        cameraIOMap.put(
                                camera,
                                new VisionIOLimelight(
                                        camera.getName(),
                                        camera.getRobotToCamera(),
                                        swerve::getRotation));
                    } else {
                        cameraIOMap.put(
                                camera,
                                new VisionIOPhotonVision(
                                        camera.getName(), camera.getRobotToCamera()));
                    }
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

                shooter = new Shooter(new ShooterIOSim(), swerve::getPose);
                hood = new Hood(new HoodIOSim(), swerve::getPose);
                intake = new Intake(new IntakeIOSim());
                feeder = new Feeder(new FeederIOSim());
                indexer = new Indexer(new IndexerIOSim());

                for (Cameras camera : Cameras.values()) {
                    cameraIOMap.put(
                            camera,
                            new VisionIOPhotonVisionSim(
                                    camera.getName(), camera.getRobotToCamera(), swerve::getPose));
                }
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

                shooter = new Shooter(new ShooterIO() {}, swerve::getPose);
                hood = new Hood(new HoodIO() {}, swerve::getPose);
                intake = new Intake(new IntakeIO() {});
                indexer = new Indexer(new IndexerIO() {});
                feeder = new Feeder(new FeederIO() {});

                for (Cameras camera : Cameras.values()) {
                    cameraIOMap.put(camera, new VisionIO() {});
                }
            }
        }
        this.vision = new Vision(swerve, cameraIOMap);

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
                "Drive Wheel Radius Characterization",
                DriveCommands.wheelRadiusCharacterization(swerve));
        autoChooser.addOption(
                "Drive Simple FF Characterization",
                DriveCommands.feedforwardCharacterization(swerve));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)",
                swerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)",
                swerve.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
                "Drive SysId (Dynamic Forward)",
                swerve.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Dynamic Reverse)",
                swerve.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // TODO: Get button mappings from driver

        controller.x().whileTrue(DriveCommands.xMode(swerve));

        controller.leftTrigger().onTrue(intake.intake());

        controller.y().onTrue(intake.stow());

        controller.rightTrigger().onTrue(intake.stopRollers());

        controller
                .leftBumper()
                .onTrue(intake.outtake().alongWith(indexer.runReverse()))
                .onFalse(intake.intake().alongWith(indexer.stop()));

        controller
                .a()
                .whileTrue(
                        Commands.sequence(
                                shooter.shoot().alongWith(hood.shoot()),
                                DriveCommands.alignToHub(swerve),
                                Commands.waitUntil(this::atShootingTolerance),
                                Commands.parallel(
                                                feeder.runForward(),
                                                indexer.runForward(),
                                                intake.stow(),
                                                DriveCommands.xMode(swerve))
                                        .until(this::shouldStopShooting),
                                DriveCommands.buzzController(controller).withTimeout(0.5)))
                .onFalse(
                        Commands.parallel(
                                feeder.stop(), indexer.stop(), intake.intake(), hood.stow()));

        controller
                .b()
                .whileTrue(
                        Commands.sequence(
                                shooter.ferry().alongWith(hood.ferry()),
                                DriveCommands.alignToFerryZone(swerve),
                                Commands.waitUntil(this::atShootingTolerance),
                                Commands.parallel(
                                                feeder.runForward(),
                                                indexer.runForward(),
                                                intake.stow(),
                                                DriveCommands.xMode(swerve))
                                        .until(this::shouldStopFerrying),
                                DriveCommands.buzzController(controller).withTimeout(0.5)))
                .onFalse(
                        Commands.parallel(
                                feeder.stop(), indexer.stop(), intake.intake(), hood.stow()));

        // KB Shot, up against the hub
        controller
                .rightBumper()
                .whileTrue(
                        Commands.sequence(
                                shooter.kb().alongWith(hood.kb()),
                                Commands.waitUntil(this::atShootingTolerance),
                                Commands.parallel(
                                        feeder.runForward(),
                                        indexer.runForward(),
                                        intake.stow(),
                                        DriveCommands.xMode(swerve))))
                .onFalse(
                        Commands.parallel(
                                feeder.stop(),
                                indexer.stop(),
                                intake.intake(),
                                shooter.shoot(),
                                hood.stow()));

        // Tower shot, up against the tower
        controller
                .povLeft()
                .whileTrue(
                        Commands.sequence(
                                shooter.tower().alongWith(hood.tower()),
                                Commands.waitUntil(this::atShootingTolerance),
                                Commands.parallel(
                                        feeder.runForward(),
                                        indexer.runForward(),
                                        intake.stow(),
                                        DriveCommands.xMode(swerve))))
                .onFalse(
                        Commands.parallel(
                                feeder.stop(),
                                indexer.stop(),
                                intake.intake(),
                                shooter.shoot(),
                                hood.stow()));
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

    private boolean atShootingTolerance() {
        return shooter.atTolerance() && hood.atTolerance();
    }

    private boolean shouldStopShooting() {
        return !swerve.isAlignedToHub() || !shooter.isShooting() || !atShootingTolerance();
    }

    private boolean shouldStopFerrying() {
        return !swerve.isAlignedToFerryZone() || !shooter.isShooting() || !atShootingTolerance();
    }
}
