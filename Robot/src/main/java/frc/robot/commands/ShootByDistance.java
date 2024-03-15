// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class ShootByDistance extends Command {
  private final DriveSubsystem dt;
  private DoubleSupplier ySup;
  private DoubleSupplier xSup;
  private DoubleSupplier rotSup;
  private final PivotSubsystem m_pivot;
 
  public ShootByDistance(DriveSubsystem dt, DoubleSupplier xSup, DoubleSupplier ySup, DoubleSupplier rotSup, PivotSubsystem pivot) {
    this.dt = dt;
    this.ySup = ySup;
    this.xSup = xSup;
    this.rotSup = rotSup;
    this.m_pivot = pivot;
    addRequirements(dt, m_pivot);
  }

  @Override
  public void initialize() {}


  @Override
  public void execute() {

    m_pivot.setGoal(1.12-(0.288*Math.log(dt.getDistToTarget())));

    dt.drive(
      applyJoystickTransform(xSup.getAsDouble()),
       applyJoystickTransform(ySup.getAsDouble()), 
       applyJoystickTransform(rotSup.getAsDouble()), 
       true,
       true
      );
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  public double applyJoystickTransform(double raw) {
    if(dt.getSlowMode()) {
      return 0.4*raw;
    }

    //raw = 0.7*raw + 0.3*(Math.pow(raw, 3));
    //return raw/1.25;
    return raw;
  }
}
