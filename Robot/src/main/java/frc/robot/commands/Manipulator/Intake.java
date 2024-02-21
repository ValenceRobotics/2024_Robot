// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Manipulator;

import frc.robot.subsystems.IntakeFeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class Intake extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem m_Manipulator;
  private final IntakeFeederSubsystem m_intakeFeeder;

  /**
   * Creates a new ExampleCommand.
   * 
   * @param subsystem The subsystem used by this command.
   */
  public Intake(ShooterSubsystem manipulator, IntakeFeederSubsystem intakeFeeder) {
    this.m_Manipulator = manipulator;
    this.m_intakeFeeder = intakeFeeder;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Manipulator, m_intakeFeeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Manipulator.setShooterPower(-0.1);
    m_intakeFeeder.setIntakeFeederPower(-0.8, -0.1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Manipulator.setShooterPower(0);
    m_intakeFeeder.setIntakeFeederPower(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
