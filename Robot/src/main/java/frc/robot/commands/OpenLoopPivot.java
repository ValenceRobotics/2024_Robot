// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;

/** An example command that uses an example subsystem. */
public class OpenLoopPivot extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final PivotSubsystem m_Arm;
  private final DoubleSupplier m_DoubleSupplier;



  /**
   * Creates a new ExampleCommand.
   * 
   * @param subsystem The subsystem used by this command.
   */
  public OpenLoopPivot(PivotSubsystem Arm, DoubleSupplier doubleSupplier) {
    this.m_Arm = Arm;
    this.m_DoubleSupplier = doubleSupplier;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Arm.setPivotPower(m_DoubleSupplier.getAsDouble());
    SmartDashboard.putNumber("Pivot/Power", m_DoubleSupplier.getAsDouble());


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Arm.setPivotPower(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
