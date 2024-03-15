package frc.robot.commands.drive.Align;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindHolonomic;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;

import frc.robot.Constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public class AlignToAmp {
        // Load the path we want to pathfind to and follow
        //TO CONFIGURE
        static PathPlannerPath path = PathPlannerPath.fromPathFile("AlignToAmp");

        // Create the constraints to use while pathfinding. The constraints defined in the path will only be used for the path.
        static PathConstraints constraints = new PathConstraints(
        4, 5.0,
        Units.degreesToRadians(180), Units.degreesToRadians(360));


        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        public static Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
        path,
        constraints,
        0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
);
}