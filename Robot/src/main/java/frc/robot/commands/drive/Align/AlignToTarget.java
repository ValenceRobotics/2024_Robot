// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.Align;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldMeasurements;

public class AlignToTarget extends Command {
  /** Creates a new AlignToTarget. */

  Translation2d target = null; 
  int delta = 100;
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

    if(!isRed){
      target = FieldMeasurements.blueTarget;
    } else{
      target = FieldMeasurements.redTarget;
    }


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      // calculate desired heading from current pose
      // reach it
      //delta = target.minus(currentPose.getTranslation()).getAngle().getDegrees();
      // if(delta > 0) {move p * error}
      // else {move -p * error}


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    // set drive to 0
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(delta) < 10;
  }
}
