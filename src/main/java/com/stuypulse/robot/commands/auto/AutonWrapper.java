package com.stuypulse.robot.commands.auto;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class AutonWrapper extends SequentialCommandGroup {
    public List<PathPlannerPath> subPaths = new ArrayList<PathPlannerPath>();

    public AutonWrapper(PathPlannerPath... paths) {
        for (PathPlannerPath path : paths) {
            subPaths.add(path);
        }
    }

    public AutonWrapper() {}

    public void logPaths() {
        for (int i = 0; i < subPaths.size(); i++) {
            Logger.recordOutput(
                    "Path Logging/path: " + subPaths.get(i).name,
                    subPaths.get(i).getPathPoses().toArray(new Pose2d[0]));
        }
    }

    public void clearFieldObjects() {
        for (int i = 0; i < subPaths.size(); i++) {
            Logger.recordOutput("Path Logging/path: " + subPaths.get(i).name, new Pose2d[0]);
        }
    }
}
