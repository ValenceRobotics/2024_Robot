// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climb;


import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.PivotSubsystem;

/** An example command that uses an example subsystem. */
public class SetClimbLeftPower extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimberSubsystem m_Climber;
  private final double m_power;



  /**
   * Creates a new ExampleCommand.
   * 
   * @param subsystem The subsystem used by this command.
   */
  public SetClimbLeftPower(ClimberSubsystem climber, double power) {
    this.m_Climber = climber;
    this.m_power = power;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  
    if (m_power == -1 && m_Climber.getExtendPosition() <= ClimberConstants.kMax1ExtendPosition) {
      m_Climber.setClimbPower(0);
    } else if (m_power == 1 && m_Climber.getExtendPosition() >= ClimberConstants.kMinExtendPosition) {
      m_Climber.setClimbPower(0);
    } else {
    m_Climber.setClimbPower(m_power);
  }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Climber.setClimbPower(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
