// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.Manipulator;

import frc.robot.subsystems.Manipulator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class Shoot extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Manipulator m_Manipulator;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Shoot(Manipulator manipulator) {
    this.m_Manipulator = manipulator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Manipulator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Manipulator.setLeftPower(0);
    m_Manipulator.setRightPower(0);
    //SmartDashboard.putNumber("Manipulator/LeftPower", 0);
    //SmartDashboard.putNumber("Manipulator/RightPower", 0);

    //SmartDashboard.putNumber("Manipulator/Power", 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Manipulator.setLeftPower(SmartDashboard.getNumber("Manipulator/LeftPower", 0));
    m_Manipulator.setRightPower(SmartDashboard.getNumber("Manipulator/RightPower", 0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Manipulator.setShooterPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
