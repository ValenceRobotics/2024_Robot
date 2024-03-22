// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.Align;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;


public class ChaseNoteCmd extends Command {

    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(5, 5);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints O_CONSTRAINTS = AutoConstants.kThetaControllerConstraints;

    //to configure
    private static final Transform3d robotToCam = new Transform3d(new Translation3d(0, 0.0, 0), new Rotation3d(0,0,0));

    private static final ProfiledPIDController xController = new ProfiledPIDController(0.15, 0, 0, X_CONSTRAINTS);
    private static final ProfiledPIDController yController = new ProfiledPIDController(0.15, 0, 0, Y_CONSTRAINTS);
    private static final ProfiledPIDController oController = new ProfiledPIDController(0.02, 0, 0.001, O_CONSTRAINTS);
    private double[] doubleArray = new double[0];


    private final DriveSubsystem m_dt;
    private DoubleSupplier ySup;
    private DoubleSupplier xSup;
    private DoubleSupplier rotSup;
    private Transform2d noteTransform;


  public ChaseNoteCmd(DriveSubsystem dt, DoubleSupplier xSup, DoubleSupplier ySup, DoubleSupplier rotSup) {
    this.m_dt = dt;

    xController.setTolerance(0.2);
    yController.setTolerance(0.2);
    oController.setTolerance(Units.degreesToRadians(3));
    addRequirements(m_dt);
  }

  @Override
  public void initialize(){


  }

  @Override
  public void execute() {
    if (SmartDashboard.getNumber("Notes Detected", 0) > 0) {
        double xDist = SmartDashboard.getNumberArray("Note 1", doubleArray)[2];
        double yDist = SmartDashboard.getNumberArray("Note 1", doubleArray)[0];

        xDist += robotToCam.getX();
        yDist += robotToCam.getY();

        noteTransform = new Transform2d(new Translation2d(xDist, yDist), new Rotation2d());

        Pose2d drivePose = m_dt.getPose();
        
        Pose2d endDrivePose = drivePose.plus(noteTransform);

        
        

        // Translation2d noteTranslation = new Translation2d(xDist, yDist);
        // Translation2d newPose = new Translation2d(0,0);
        // double theta = noteTranslation.minus(newPose).getAngle().getDegrees();

        //m_dt.drive(xController.calculate(xDist, 0), yController.calculate(yDist, 0), 0, false, true);

        m_dt.drive(xController.calculate(endDrivePose.getX()-m_dt.getPose().getX(),0), yController.calculate(endDrivePose.getY()-m_dt.getPose().getY(),0), 0, true, true);

        // double theta = (Units.degreesToRadians(270+m_dt.getPose().getRotation().getDegrees()));

        // double yAdjust = yController.calculate(yDist);
        // double xAdjust = Math.sin(theta) * yAdjust;
        // yAdjust = Math.cos(theta) * yAdjust;
        // m_dt.drive(controller.calculate)

        
    } else {
        m_dt.drive(
            applyJoystickTransform(xSup.getAsDouble()),
             applyJoystickTransform(ySup.getAsDouble()), 
             applyJoystickTransform(rotSup.getAsDouble()), 
             true,
             true
            );
    }


    


  }

  @Override
  public boolean isFinished() {
    return false;
  }


  public double applyJoystickTransform(double raw) {
    if(m_dt.getSlowMode()) {
      return 0.4*raw;
    }

    //raw = 0.7*raw + 0.3*(Math.pow(raw, 3));
    //return raw/1.25;
    return raw;
  }
}