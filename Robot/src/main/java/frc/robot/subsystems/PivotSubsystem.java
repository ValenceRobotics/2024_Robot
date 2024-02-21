// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PivotSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

    private final CANSparkMax pivotMotor1;
    private final CANSparkMax pivotMotor2;
    private final ProfiledPIDController pivotController;
    private final ArmFeedforward pivotFeedforward;
    private final DutyCycleEncoder absEncoder;
    private final double pivotVelocity;
    //abs encoder
    //possible feedforward?

  public PivotSubsystem() {
    pivotMotor1 = createPivotController(PivotConstants.pivotMotor1Id, false);
    pivotMotor2 = createPivotController(PivotConstants.pivotMotor2Id, true);
    pivotController = new ProfiledPIDController(0, 0, 0, PivotConstants.kPivotControllerConstraints);
    pivotFeedforward = new ArmFeedforward(0, 0, 0);
    absEncoder = new DutyCycleEncoder(0);
    absEncoder.setPositionOffset(PivotConstants.positionOffset);
    pivotVelocity = 0.5;

  }


  //set position method
  //get position method
  
  public void setPivotPosition(double position) {
    double pivotFeed = pivotFeedforward.calculate(position, pivotVelocity);
    double pidFeed = pivotController.calculate(getPivotPosition(), position);
    setPivotVolts(pivotFeed + pidFeed);
  }

  public double getPivotPosition() {
    return (absEncoder.getAbsolutePosition()-absEncoder.getPositionOffset()) * 2 * Math.PI;
  }



  public void setPivotPower(double power) {
    pivotMotor1.set(power);
    pivotMotor2.set(power);
  }

  public void setPivotVolts(double voltage) {
    pivotMotor1.setVoltage(voltage);
    pivotMotor2.setVoltage(voltage);
  }

  public void setLeftPower(double power) {
    pivotMotor1.set(power);
  }

  public void setRightPower(double power) {
    pivotMotor2.set(power);
  }



  private CANSparkMax createPivotController(int port, boolean isInverted) {
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
