// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkFlex;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

    //private final CANSparkFlex shooterMotor1;
    //private final CANSparkFlex shooterMotor2;
    private final TalonFX shooterMotor1;
    private final TalonFX shooterMotor2;
    private final PIDController shooterPid;
    private final SimpleMotorFeedforward shooterFeedforward;
    //ideally would want a velocity controller




  public ShooterSubsystem() {
    //shooterMotor1 = createShooterController(ShooterConstants.shooterMotor1Id, false);
    //shooterMotor2 = createShooterController(ShooterConstants.shooterMotor2Id, true);
    shooterMotor1 = createFalconShooterController(ShooterConstants.shooterMotor1Id, false);
    shooterMotor2 = createFalconShooterController(ShooterConstants.shooterMotor2Id, true);
    shooterPid = new PIDController(0, 0, 0);
    shooterFeedforward = new SimpleMotorFeedforward(0, 0);





  }

  public void setShooterPower(double power) {
    shooterMotor1.set(power);
    shooterMotor2.set(power);
  }

  public void setLeftPower(double power) {
    shooterMotor1.set(power);
  }

  public void setRightPower(double power) {
    shooterMotor2.set(power);
  }


  public void setShooterVelocity(double velocity) {
    double feedVelocity = shooterFeedforward.calculate(velocity);
    double pidVelocity1 = shooterPid.calculate(getTopVelocityRPM(), velocity);
    double pidVelocity2 = shooterPid.calculate(getBottomVelocityRPM(), velocity);
    shooterMotor1.setVoltage(feedVelocity + pidVelocity1);
    shooterMotor2.setVoltage(feedVelocity + pidVelocity2);
  }

  public double getTopVelocityRPM() {
    return shooterMotor1.get() * ShooterConstants.vortexRPM;
  }

  public double getBottomVelocityRPM() {
    return shooterMotor2.get() * ShooterConstants.vortexRPM;
  }



  private CANSparkMax createManipulatorController(int port, boolean isInverted) {
    CANSparkMax controller = new CANSparkMax(port, MotorType.kBrushless);
    controller.restoreFactoryDefaults();

    controller.setIdleMode(CANSparkMax.IdleMode.kBrake);

    controller.setInverted(isInverted);

    return controller;
}

  private CANSparkFlex createShooterController(int port, boolean isInverted) {
    CANSparkFlex controller = new CANSparkFlex(port, MotorType.kBrushless);
    controller.restoreFactoryDefaults();

    controller.setIdleMode(CANSparkFlex.IdleMode.kBrake);

    controller.setInverted(isInverted);

    return controller;
  }

  private TalonFX createFalconShooterController(int id, boolean invert) {
    TalonFX shooterController = new TalonFX(id);
    //shooterController.configFactoryDefault();

    shooterController.setInverted(invert);
    shooterController.setNeutralMode(ShooterConstants.NEUTRAL_MODE);
    // shooterController.configSupplyCurrentLimit(ShooterConstants.CURRENT_LIMIT);

    // shooterController.configOpenloopRamp(ShooterConstants.LONG_RAMP_RATE);
    // shooterController.configClosedloopRamp(ShooterConstants.LONG_RAMP_RATE);
    // shooterController.configVoltageCompSaturation(ShooterConstants.VOLTAGE_COMPENSATION);
    //shooterController.enableVoltageCompensation(true);

    return shooterController;
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
