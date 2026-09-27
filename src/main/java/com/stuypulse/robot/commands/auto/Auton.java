package com.stuypulse.robot.commands.auto;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.constants.Field;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.ArrayList;
import java.util.List;
import com.stuypulse.robot.RobotContainer;

/**
 * <h2> Auton </h2>
 * 
 * <p> Made for the purpose of previewing the selected auton in elastic
 * <p> The selected path disappears when a different auton is selected or when the robot is enabled
 * 
 * <p> Define an auton in {@link AutonFactory} and add it to the auton chooser as shown in the example auton in {@link RobotContainer}
 * 
 */
public class Auton extends SequentialCommandGroup {
    private final List<PathPlannerPath> subPaths = new ArrayList<PathPlannerPath>();

    /**
     * Creates an auton that previews the given paths and runs the given commands
     * @param paths Paths to preview
     * @param commands Auton command seqeunce
     */
    public Auton(PathPlannerPath[] paths, Command... commands) {
        for (PathPlannerPath path : paths) {
            subPaths.add(path);
        }

        addCommands(commands);
    }

    /**
     * Creates an auton without paths, for sysid and characterization routines
     * @param command The command to run
     */
    public Auton(Command command) {
        addCommands(command);
    }

    /**
     * Creates an empty auton used for the do nothing auton
     */
    public Auton() {}

    /**
     * Sets path poses as field objects which display as trajectories on elastic
     */
    public void logPaths() {

        for (int i = 0; i < subPaths.size(); i++) {
            if (DriverStation.getAlliance().isEmpty()) {
                Field.FIELD_2D
                        .getObject("path: " + subPaths.get(i).name)
                        .setPoses(
                                Field.transformToOppositeAlliance(subPaths.get(i).getPathPoses()));
                continue;
            }

            if (DriverStation.getAlliance().get() == Alliance.Blue) {
                Field.FIELD_2D
                        .getObject("path: " + subPaths.get(i).name)
                        .setPoses(subPaths.get(i).getPathPoses());
            } else {
                Field.FIELD_2D
                        .getObject("path: " + subPaths.get(i).name)
                        .setPoses(
                                Field.transformToOppositeAlliance(subPaths.get(i).getPathPoses()));
            }
        }
    }

    /**
     * Sets field objects to empty poses so they disappear
     */
    public void clearFieldObjects() {
        for (int i = 0; i < subPaths.size(); i++) {
            Field.FIELD_2D.getObject("path: " + subPaths.get(i).name).setPoses();
        }
    }
}
