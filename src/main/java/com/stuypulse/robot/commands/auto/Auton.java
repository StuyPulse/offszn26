package com.stuypulse.robot.commands.auto;

import com.pathplanner.lib.path.PathPlannerPath;
import com.stuypulse.robot.constants.Field;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.ArrayList;
import java.util.List;

public class Auton extends SequentialCommandGroup {
    private static final List<Auton> loggedAutons = new ArrayList<Auton>();

    private final List<PathPlannerPath> subPaths = new ArrayList<PathPlannerPath>();

    public Auton(PathPlannerPath[] paths, Command... commands) {
        loggedAutons.add(this);

        for (PathPlannerPath path : paths) {
            subPaths.add(path);
        }

        addCommands(commands);
    }

    // Used for sysid or charcterization routines so that they can be added to the auton chooser
    public Auton(Command command) {
        addCommands(command);
    }

    public Auton() {}

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

    public void clearFieldObjects() {
        for (int i = 0; i < subPaths.size(); i++) {
            Field.FIELD_2D.getObject("path: " + subPaths.get(i).name).setPoses(new ArrayList<>());
        }
    }

    private void closeFieldObjects() {
        for (int i = 0; i < subPaths.size(); i++) {
            Field.FIELD_2D.getObject("path: " + subPaths.get(i).name).setPoses(new ArrayList<>());
            Field.FIELD_2D.getObject("path: " + subPaths.get(i).name).close();
        }
    }

    public static void closeAllFieldObjects() {
        for (Auton auton : loggedAutons) {
            auton.closeFieldObjects();
        }
    }
}
