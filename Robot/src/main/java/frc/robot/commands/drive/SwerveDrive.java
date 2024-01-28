// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class SwerveDrive extends Command {
  private final DriveSubsystem dt;
  private DoubleSupplier ySup;
  private DoubleSupplier xSup;
  private DoubleSupplier rotSup;
 
  public SwerveDrive(DriveSubsystem dt, DoubleSupplier xSup, DoubleSupplier ySup, DoubleSupplier rotSup) {
    this.dt = dt;
    this.ySup = ySup;
    this.xSup = xSup;
    this.rotSup = rotSup;
    addRequirements(dt);
  }

  @Override
  public void initialize() {}


  @Override
  public void execute() {
    dt.drive(
      applyJoystickTransform(xSup.getAsDouble()),
       applyJoystickTransform(ySup.getAsDouble()), 
       applyJoystickTransform(rotSup.getAsDouble()), 
       dt.getFieldRelative(), 
       true
      );
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  public double applyJoystickTransform(double raw) {
    if(dt.getSlowMode()) {
      return 0.2*raw;
    }

    //raw = 0.7*raw + 0.3*(Math.pow(raw, 3));
    return raw/1.25;
    // return raw;
  }
}