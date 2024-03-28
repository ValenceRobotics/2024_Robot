// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeState;
import frc.robot.Constants.ShooterState;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeFeederSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoShoot extends Command {

  private final ShooterSubsystem m_shooter;
  private final IntakeFeederSubsystem m_feeder;
 
  public AutoShoot( ShooterSubsystem shooter, IntakeFeederSubsystem feeder) {

    this.m_shooter = shooter;
    this.m_feeder = feeder;
    addRequirements(m_shooter, m_feeder);
  }

  @Override
  public void initialize() {
    m_shooter.setShooterState(ShooterState.SHOOTING);
  }


  @Override
  public void execute() {
   if (m_shooter.getTopVelocity() > 6400 && m_shooter.getBottomVelocity() > 5600) {
    m_feeder.setIntakeState(IntakeState.SHOOTING);
   } else {
    m_feeder.setIntakeState(IntakeState.STOPPED);
   }
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}
