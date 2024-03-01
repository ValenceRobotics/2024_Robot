// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.Align;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DebugConstants;
import frc.robot.Constants.FieldMeasurements;
import frc.robot.subsystems.DriveSubsystem;

public class AlignToTarget extends Command {
  /** Creates a new AlignToTarget. */
  DriveSubsystem m_Drive;
  PIDController thetaController = new PIDController(0.0015, 0, 0);

  Translation2d target = null; 
  double delta = 100;
  public AlignToTarget(DriveSubsystem drive) {
    this.m_Drive = drive;
    thetaController.enableContinuousInput(-180, 180);
    addRequirements(m_Drive);
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
      delta = target.minus(m_Drive.getPose().getTranslation()).getAngle().getDegrees();

    if(DebugConstants.kDebugMode){

      SmartDashboard.putNumber("AutoAlign Error", delta);
    }

      m_Drive.drive(0, 0, -thetaController.calculate(delta,0), false, true);



  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    // set drive to 0
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(delta) < 3;
  }
}
