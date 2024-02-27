// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;

/** An example command that uses an example subsystem. */
public class PivotPID extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final PivotSubsystem m_Arm;
  private final double m_DoubleSupplier;
  private final PIDController armController = new PIDController(0.1, 0, 0.0);
  double gravConst;


      //private final ArmFeedforward armFeedforward = new ArmFeedforward(0,6,0,0);





  /**
   * Creates a new ExampleCommand.
   * 
   * @param subsystem The subsystem used by this command.
   */
  public PivotPID(PivotSubsystem Arm, double doubleSupplier) {
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
   if (m_Arm.getPivotPosition()>1.9) {
    gravConst = 0.12; // old 0.15
    m_Arm.setPivotPower((armController.calculate(m_Arm.getPivotPosition(), m_DoubleSupplier))+(gravConst*Math.cos(m_Arm.getPivotPosition()-0.1)));


   } else {
    gravConst = 0.0783976825;
    m_Arm.setPivotPower((armController.calculate(m_Arm.getPivotPosition(), m_DoubleSupplier))+(gravConst*Math.cos(m_Arm.getPivotPosition()-0.1)));

   };
    //m_Arm.setPivotVolts(armController.calculate(m_Arm.getPivotPosition(), m_DoubleSupplier)  
    //   + armFeedforward.calculate(m_Arm.getPivotPosition(), m_DoubleSupplier));

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
