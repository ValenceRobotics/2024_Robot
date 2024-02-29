// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Manipulator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeState;
import frc.robot.Constants.ShooterState;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeFeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/** An example command that uses an example subsystem. */
public class SetMechanismState extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})


  /**
   * Creates a new ExampleCommand.
   * 
   * @param subsystem The subsystem used by this command.
   */

  IntakeState intakeState = IntakeState.STOPPED;
  ShooterState shooterState = ShooterState.STOPPED;
   public SetMechanismState(IntakeState intakeState, ShooterState shooterState) {
    this.intakeState = intakeState;
    this.shooterState = shooterState;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_IntakeFeederSubsystem, RobotContainer.m_Shooter);
  }

  boolean onlyIntake = false; 
  boolean onlyShooter = false;
  public SetMechanismState(IntakeState intakeState) {
      this.onlyIntake = true; 
    this.intakeState = intakeState;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_IntakeFeederSubsystem, RobotContainer.m_Shooter);
  }

    public SetMechanismState(ShooterState shooterState) {
      this.onlyShooter = true; 
    this.shooterState = shooterState;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_IntakeFeederSubsystem, RobotContainer.m_Shooter);
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(onlyIntake){
      RobotContainer.m_IntakeFeederSubsystem.setIntakeState(this.intakeState);

    } else if(onlyShooter){
      RobotContainer.m_Shooter.setShooterState(this.shooterState);


    } else {
            RobotContainer.m_IntakeFeederSubsystem.setIntakeState(this.intakeState);
      RobotContainer.m_Shooter.setShooterState(this.shooterState);


    }





  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  

    // m_Manipulator.setShooterPower(-0.5, -0.5);
    // m_intakeFeeder.setIntakeFeederPower(-0.7, -0.6);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;  // return m_intakeFeeder.hasNote();
  }
}
