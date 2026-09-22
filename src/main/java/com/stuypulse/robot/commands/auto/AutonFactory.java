package com.stuypulse.robot.commands.auto;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.subsystems.feeder.Feeder;
import com.stuypulse.robot.subsystems.hood.Hood;
import com.stuypulse.robot.subsystems.indexer.Indexer;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.swerve.Swerve;
import com.stuypulse.robot.subsystems.vision.Vision;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutonFactory {
    // Define all auton commands here

    private final Feeder feeder;
    private final Hood hood;
    private final Indexer indexer;
    private final Intake intake;
    private final Shooter shooter;
    private final Swerve swerve;
    private final Vision vision;

    public AutonFactory(
            Feeder feeder,
            Hood hood,
            Indexer indexer,
            Intake intake,
            Shooter shooter,
            Swerve swerve,
            Vision vision) {
        this.feeder = feeder;
        this.hood = hood;
        this.indexer = indexer;
        this.intake = intake;
        this.shooter = shooter;
        this.swerve = swerve;
        this.vision = vision;
    }

    public Command doNothingAuton() {
        return Commands.sequence(
                // Does nothing
                );
    }

    public Command exampleAuton(PathPlannerPath... paths) {
        return Commands.sequence(swerve.followPath(paths[0]));
    }
}
