// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.PivotSubsystem;

/** An example command that uses an example subsystem. */
public class SetPivotPosition extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final PivotSubsystem m_Arm;
  private double m_GoalPosition;
  private final PIDController armController = new PIDController(0.1, 0, 0.0);
  double gravConst;


      //private final ArmFeedforward armFeedforward = new ArmFeedforward(0,6,0,0);





  /**
   * Creates a new ExampleCommand.
   * 
   * @param subsystem The subsystem used by this command.
   */
  public SetPivotPosition(PivotSubsystem Arm, double goalPosition) {
    this.m_Arm = Arm;
    this.m_GoalPosition = goalPosition;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Arm);
  }
  boolean isTest = false; 
    public SetPivotPosition(PivotSubsystem Arm, DoubleSupplier goalPosition) {
    this.m_Arm = Arm;
    this.m_GoalPosition = goalPosition.getAsDouble();
    isTest = true; 
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Arm);
  }
boolean calculate = false;
    public SetPivotPosition(PivotSubsystem Arm,boolean calculate) {
      this.calculate = true; 
    this.m_Arm = Arm;
    isTest = false; 
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Arm);
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  if(isTest){

  m_GoalPosition = SmartDashboard.getNumber("test pivot loc", PivotConstants.kHomePosition);
    
  }else if (calculate) {

    double m = RobotContainer.m_robotDrive.getDistToTarget();
    double j = 1.47 - 0.34*m + 0.0359*m*m;
    m_GoalPosition = j;
  }
  m_Arm.setGoal(m_GoalPosition);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Arm.atGoal();
  }
}
