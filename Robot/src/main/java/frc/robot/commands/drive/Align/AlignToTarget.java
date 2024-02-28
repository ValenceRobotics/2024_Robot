// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.Align;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

public class AlignToTarget extends Command {
  /** Creates a new AlignToTarget. */

  Pose2d target = null; 
  public AlignToTarget() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    var alliance = DriverStation.getAlliance();
    boolean isRed = false; 
    if (alliance.isPresent()) {
      isRed = alliance.get() == DriverStation.Alliance.Red;
    }

    if(isRed){
      target = new Pose2d(0.25, 5.53, new Rotation2d(0));
    } else{
      target = new Pose2d(16.29, 5.53, new Rotation2d(0));
    }


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      // calculate desired heading from current pose
      // reach it


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
