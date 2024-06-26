package frc.robot.commands.drive.Align;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindHolonomic;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;

import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FieldMeasurements;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


// public class AlignToAmp {
//         // Load the path we want to pathfind to and follow
//         //TO CONFIGURE
//         static PathPlannerPath path = PathPlannerPath.fromPathFile("AlignToAmp");

//         // Create the constraints to use while pathfinding. The constraints defined in the path will only be used for the path.
//         static PathConstraints constraints = new PathConstraints(
//         4, 5.0,
//         Units.degreesToRadians(180), Units.degreesToRadians(360));


//         // Since AutoBuilder is configured, we can use it to build pathfinding commands
//         public static Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
//         path,
//         constraints,
//         0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
// );
// }

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.




public class AlignToAmp extends Command {

    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(4.5, 6);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(4.5, 6);
    private static final TrapezoidProfile.Constraints O_CONSTRAINTS = AutoConstants.kThetaControllerConstraints;

    //to configure
    private static final ProfiledPIDController xController = new ProfiledPIDController(0.32, 0, 0, X_CONSTRAINTS);
    private static final ProfiledPIDController yController = new ProfiledPIDController(0.32, 0, 0, Y_CONSTRAINTS);
    private static final PIDController oController = new PIDController(0.032, 0, 0.001);
    private double currentAngle;
    


    private final DriveSubsystem m_dt;
    boolean isRed;
    Pose2d target = null; 

    


  public AlignToAmp(DriveSubsystem dt) {
    this.m_dt = dt;

    xController.setTolerance(0.02);
    yController.setTolerance(0.02);
    oController.setTolerance(3);
    currentAngle =angleWrap(dt.getHeading());
    this.currentAngle = angleWrap(dt.getHeading());
    addRequirements(m_dt);
  }

  @Override
  public void initialize(){

    //double desiredAngle = angleWrap(270);
    oController.enableContinuousInput(-180, 180);

    var alliance = DriverStation.getAlliance();
    isRed = false; 
    if (alliance.isPresent()) {
      isRed = alliance.get() == DriverStation.Alliance.Red;
    }

    if(!isRed){
      target = FieldMeasurements.blueLineupAmp;
    } else{
      target = FieldMeasurements.redLineupAmp;
    }

  }

  @Override
  public void execute() {

        double xDist = target.getX()-m_dt.getPose().getX();
        double yDist = target.getY()-m_dt.getPose().getY();

        SmartDashboard.putString("Amp align/ target", target.toString());

        if (!isRed) {
        xDist *= -1;
        yDist *= -1;
        }


        SmartDashboard.putNumber("Amp align/ xDist", xDist);
        SmartDashboard.putNumber("Amp align/ yDist", xDist);

        currentAngle = angleWrap(m_dt.getHeading());
        double desiredAngle = angleWrap(270);
        double output = oController.calculate(currentAngle, desiredAngle);

        m_dt.drive(xController.calculate(xDist, 0), yController.calculate(yDist, 0), output, true, true);

  }

  @Override
  public boolean isFinished() {
    return false;
  }


  public double angleWrap(double angle) {
    double radians = Math.toRadians(angle);
    while (radians > Math.PI) {
      radians -= 2 * Math.PI;
    }
    while (radians < -Math.PI) {
      radians += 2 * Math.PI;
    }

    return Math.toDegrees(radians);
  }

}
