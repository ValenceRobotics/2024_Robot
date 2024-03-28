// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.Align;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DebugConstants;
import frc.robot.Constants.FieldMeasurements;
import frc.robot.subsystems.DriveSubsystem;

public class AlignToTargetAuto extends Command {
  /** Creates a new AlignToTarget. */
  DriveSubsystem m_Drive;
  PIDController thetaController = new PIDController(0.01, 0, 0);
  boolean isRed;
  Pose2d target = null; 
  DoubleSupplier xSup;
  DoubleSupplier ySup;
  double delta = 100;
  public AlignToTargetAuto(DriveSubsystem drive) {
    this.m_Drive = drive;
    thetaController.enableContinuousInput(-180, 180);
    addRequirements(m_Drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    var alliance = DriverStation.getAlliance();
    isRed = false; 
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
      var targetRotationPose = target.getTranslation().minus(m_Drive.getPose().getTranslation()).getAngle();
      
      var delta = m_Drive.getPose().getRotation().minus(targetRotationPose).getDegrees();



      m_Drive.drive(0,0, thetaController.calculate(delta,0), true, true);



  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return thetaController.atSetpoint();

  }
}
