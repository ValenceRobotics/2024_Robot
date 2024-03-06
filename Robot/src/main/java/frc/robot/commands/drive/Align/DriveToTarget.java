// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.Align;

import java.util.List;
import java.util.stream.Collectors;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.FieldMeasurements;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToTarget extends InstantCommand {
  public DriveToTarget() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_robotDrive);
  }
  Command currCommand = null; 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

     List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
        RobotContainer.m_robotDrive.getPose(), (RobotContainer.isRed ? FieldMeasurements.redLineupAmp : FieldMeasurements.blueLineupAmp)
    );
    PathPlannerPath path = new PathPlannerPath(bezierPoints, new PathConstraints(3.0,3.0,2*Math.PI, 4*Math.PI)
    , new GoalEndState(0.0, new Rotation2d(Units.degreesToRadians(90)))
    
    );

    path.preventFlipping = true;
    RobotContainer.currentAlignPath = AutoBuilder.followPath(path);


    List<Pose2d> poses = path.getAllPathPoints().stream().map(point-> new Pose2d(point.position, new Rotation2d(0))).collect(Collectors.toList());
    RobotContainer.m_robotDrive.m_field.getObject("path").setPoses(poses);
    
    CommandScheduler.getInstance().schedule(RobotContainer.currentAlignPath);



  }

//   @Override
//   public void execute() {
//     this.currCommand.execute();

//   }


//   @Override
//   public void end(boolean interrupted) {

// =  }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return this.currCommand != null && currCommand.isFinished();
//   }
}
