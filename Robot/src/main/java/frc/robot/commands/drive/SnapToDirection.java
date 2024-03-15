// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;


public class SnapToDirection extends Command {

  private final DriveSubsystem dt;
  private double desiredAngle;
  private final DoubleSupplier xSup;
  private final DoubleSupplier ySup;
  private PIDController controller;
  private double currentAngle;
  private double output;

  public SnapToDirection(DriveSubsystem dt, double angle, DoubleSupplier xSup, DoubleSupplier ySup) {
    this.dt = dt;
    this.desiredAngle = angle;
    this.controller = new PIDController(0.025, 0, 0.001);
    this.xSup = xSup;
    this.ySup = ySup;
    currentAngle =angleWrap(dt.getHeading());
    this.currentAngle = angleWrap(dt.getHeading());

    addRequirements(dt);
  }

  @Override
  public void initialize(){
    desiredAngle = angleWrap(desiredAngle);
    controller.enableContinuousInput(-180, 180);
  }

  @Override
  public void execute() {
    currentAngle = angleWrap(dt.getHeading());
    desiredAngle = angleWrap(desiredAngle);
    output = controller.calculate(currentAngle, desiredAngle);


    dt.drive(applyJoystickTransform(xSup.getAsDouble()), applyJoystickTransform(ySup.getAsDouble()), output,
        true, true);

  }

  @Override
  public boolean isFinished() {
    return Math.abs(currentAngle  - desiredAngle) < 5; // 4 degrees
  }

  public double applyJoystickTransform(double raw) {
    if (dt.getSlowMode()) {
      return 0.2 * raw;
    }

    // raw = 0.7*raw + 0.3*(Math.pow(raw, 3));
    return raw / 2;
  }

  // This function normalizes the angle so it returns a value between -180° and
  // 180° instead of 0° to 360°.
  public double angleWrap(double angle) {
    double radians = Math.toRadians(angle);
    while (radians > Math.PI) {
      radians -= 2 * Math.PI;
    }
    while (radians < -Math.PI) {
      radians += 2 * Math.PI;
    }

    return Math.toDegrees(radians);
  }

  public double getOutput(){
    return output;
  }

}