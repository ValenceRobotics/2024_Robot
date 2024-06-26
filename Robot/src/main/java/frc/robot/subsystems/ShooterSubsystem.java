// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DebugConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterState;
import frc.robot.Constants.ShooterConstants;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkFlex;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.MutableMeasure.mutable;
import edu.wpi.first.units.MutableMeasure;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

    private final CANSparkFlex shooterMotortop;

    private final CANSparkFlex shooterMotorbottom;
    //private final TalonFX shooterMotortop;
    //private final TalonFX shooterMotorbottom;
    private final PIDController shooterPid;
    private final SimpleMotorFeedforward shooterFeedforward;
    //ideally would want a velocity controller



  private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
    new SysIdRoutine.Config(),
    new SysIdRoutine.Mechanism(
    (voltage) -> this.setShooterVolts(voltage.in(Volts)),
     null, // No log consumer, since data is recorded by URCL
    this
  ));

  private ShooterState currentState = ShooterState.STOPPED;
  public ShooterSubsystem() {

    shooterMotortop = createShooterController(ShooterConstants.shooterMotortopId, false);
    shooterMotorbottom = createShooterController(ShooterConstants.shooterMotorbottomId, false);

    //shooterMotortop = createFalconShooterController(ShooterConstants.shooterMotor1Id, false);
   // shooterMotorbottom = createFalconShooterController(ShooterConstants.shooterMotor2Id, true);
    shooterPid = new PIDController(0, 0, 0);
    shooterFeedforward = new SimpleMotorFeedforward(0, 0);





  }

  public void setShooterState(ShooterState state){
    this.currentState = state; 
  }



  public void setShooterPower(double toppower, double bottompower) {
    shooterMotortop.set(toppower);
    shooterMotorbottom.set(bottompower);



  }

  public void setLeftPower(double power) {
    shooterMotortop.set(power);
  }

  public void setRightPower(double power) {
    shooterMotorbottom.set(power);
  }

  public void setShooterVolts(double volts) {
    shooterMotortop.setVoltage(volts);
    shooterMotorbottom.setVoltage(volts);
  }


  public void setShooterVelocity(double velocity) {
    double feedVelocity = shooterFeedforward.calculate(velocity);
    double pidVelocity1 = shooterPid.calculate(getTopVelocity(), velocity);
    double pidVelocity2 = shooterPid.calculate(getBottomVelocity(), velocity);
    shooterMotortop.setVoltage(feedVelocity + pidVelocity1);
    shooterMotorbottom.setVoltage(feedVelocity + pidVelocity2);
  }

  public double getTopVelocity() {
    return shooterMotortop.getEncoder().getVelocity();
  }

  public double getBottomVelocity() {
    return shooterMotorbottom.getEncoder().getVelocity();
  }




  private CANSparkFlex createShooterController(int port, boolean isInverted) {
    CANSparkFlex controller = new CANSparkFlex(port, MotorType.kBrushless);
    controller.restoreFactoryDefaults();

    controller.setIdleMode(CANSparkFlex.IdleMode.kCoast);
    controller.setSmartCurrentLimit(40);
    controller.setClosedLoopRampRate(0.0);
    controller.setOpenLoopRampRate(0.0);


    controller.setInverted(isInverted);

    //controller.burnFlash();

    return controller;
  }


  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

/**
 * Returns a command that will execute a dynamic test in the given direction.
 *
 * @param direction The direction (forward or reverse) to run the test in
 */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
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
      shooterMotorbottom.set(currentState.lowSpeed);
      shooterMotortop.set(currentState.highSpeed);


   if (DebugConstants.kDebugMode) {

    SmartDashboard.putNumber("shooter motor 1", this.getTopVelocity());
    SmartDashboard.putNumber("shooter motor 2", this.getBottomVelocity());
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
