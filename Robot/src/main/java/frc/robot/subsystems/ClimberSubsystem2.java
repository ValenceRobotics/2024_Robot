// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.PivotConstants;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimberSubsystem2 extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

    //private final CANSparkMax climbMotor1;
    private final CANSparkMax climbMotor2;
    //string pot

  public ClimberSubsystem2() {
    //climbMotor1 = createClimberController(ClimberConstants.climbMotor1Id, false);
    climbMotor2 = createClimberController(ClimberConstants.climbMotor2Id, false);


  }




  public void setClimbPower(double power) {
    //climbMotor1.set(power);
    climbMotor2.set(power);
  }

  public void setLeftPower(double power) {
    //climbMotor1.set(power);
  }

  public void setRightPower(double power) {
    climbMotor2.set(power);
  }

  public double getExtendPosition() {
    return climbMotor2.getEncoder().getPosition();
  }



  private CANSparkMax createClimberController(int port, boolean isInverted) {
    CANSparkMax controller = new CANSparkMax(port, MotorType.kBrushless);
    controller.restoreFactoryDefaults();

    controller.setIdleMode(CANSparkMax.IdleMode.kBrake);

    controller.setInverted(isInverted);

    return controller;
}

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    //SmartDashboard.putData(shooterPower);
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
