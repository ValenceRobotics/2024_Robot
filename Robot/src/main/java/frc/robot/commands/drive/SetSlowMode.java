// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class SetSlowMode extends Command {

  private final DriveSubsystem dt;
  private final boolean slowMode;
  public SetSlowMode(DriveSubsystem dt, boolean slowMode) {
    this.dt = dt;
    this.slowMode = slowMode;
  }

  @Override
  public void initialize() {
    dt.setSlowMode(slowMode);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}