// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Manipulator;

import frc.robot.subsystems.IntakeFeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.List;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class Shoot extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeFeederSubsystem m_IntakeFeeder;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Shoot(IntakeFeederSubsystem intakeFeeder) {
    this.m_IntakeFeeder = intakeFeeder;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_IntakeFeeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
<<<<<<< HEAD
    m_IntakeFeeder.setFeederPower(0.2);

//         // speaker apriltags: 3,4,7,8
// /**/
//     AprilTagProcessor myAprilTagProcessor;
//     List<AprilTagDetection> myAprilTagDetections;  // list of all detections
//     AprilTagDetection myAprilTagDetection;         // current detection in for() loop
//     int myAprilTagIdCode;                           // ID code of current detection, in for() loop

//     // Get a list of AprilTag detections.
//     myAprilTagDetections = myAprilTagProcessor.getDetections();
//     // Cycle through through the list and process each AprilTag.
//     for (myAprilTagDetection : myAprilTagDetections) {
//       if (myAprilTagDetection.metadata != null) {  // This check for non-null Metadata is not needed for reading only ID code.
//         myAprilTagIdCode = myAprilTagDetection.id;
//         // Now take action based on this tag's ID code, or store info for later action.
//         switch (myAprilTagIdCode) {
//           case 3:
//           case 4:
//           case 7:
//           case 8:
//             m_Manipulator.setShooterPower(1.0);
//             break;
//         }
//       }
//     }

=======
    m_Manipulator.setLeftPower(SmartDashboard.getNumber("Manipulator/LeftPower", 0));
    m_Manipulator.setRightPower(SmartDashboard.getNumber("Manipulator/RightPower", 0));

    // speaker apriltags: 3,4,7,8
/**/
    AprilTagProcessor myAprilTagProcessor;
    List<AprilTagDetection> myAprilTagDetections;  // list of all detections
    AprilTagDetection myAprilTagDetection;         // current detection in for() loop
    int myAprilTagIdCode;                           // ID code of current detection, in for() loop

    // Get a list of AprilTag detections.
    myAprilTagDetections = myAprilTagProcessor.getDetections();
    // Cycle through through the list and process each AprilTag.
    for (myAprilTagDetection : myAprilTagDetections) {
      if (myAprilTagDetection.metadata != null) {  // This check for non-null Metadata is not needed for reading only ID code.
        myAprilTagIdCode = myAprilTagDetection.id;
        // Now take action based on this tag's ID code, or store info for later action.
        switch (myAprilTagIdCode) {
          case 3:
          case 4:
          case 7:
          case 8:
            m_Manipulator.setShooterPower(1.0);
            break;
        }
      }
    }
/**/
>>>>>>> 6eb8cd075c85886ce6ef19cf6082736631e99dbe
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeFeeder.setFeederPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
