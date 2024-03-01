// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DebugConstants;
import frc.robot.Constants.IntakeFeederConstants;
import frc.robot.Constants.IntakeState;

public class IntakeFeederSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

    private final CANSparkMax intakeMotor;
    private final CANSparkMax feederMotor1;
    private boolean hasNote = true; 
    private final double NOTE_HELD_THRESHOLD = 3000;


  public IntakeFeederSubsystem() {
    intakeMotor = createIntakeFeederController(IntakeFeederConstants.intakeMotorId, false);
    feederMotor1 = createIntakeFeederController(IntakeFeederConstants.feederMotor1Id, false);

  }

  public boolean hasNote() {
    return this.hasNote;
  }

  public void setNote(boolean hasNote) {
    this.hasNote = hasNote;
  }



  public void setIntakePower(double power) {
    intakeMotor.set(power);
  }

  private IntakeState currentState = IntakeState.STOPPED;

  public void setIntakeState(IntakeState state){
    this.currentState = state; 
  }

  public void setFeederPower(double power) {
    feederMotor1.set(power); 
  }

  public void setIntakeFeederPower(double intakePower, double feederPower) {
    intakeMotor.set(intakePower);
    setFeederPower(feederPower);
  }


  private CANSparkMax createIntakeFeederController(int port, boolean isInverted) {
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
    if (DebugConstants.kDebugMode) {
    SmartDashboard.putNumber("feeder Velocity", Math.abs(feederMotor1.getEncoder().getVelocity()));
    SmartDashboard.putBoolean("hasNote", this.hasNote);
    }
    
    if(Math.abs(feederMotor1.get()) > 0 && Math.abs(feederMotor1.getEncoder().getVelocity()) < NOTE_HELD_THRESHOLD){
      this.hasNote = true; 
    } else if(Math.abs(feederMotor1.get()) > 0) {  // implicitly means our velocity is greater
      this.hasNote = false; 
    }


    this.intakeMotor.set(-currentState.intakeSpeed);
    this.feederMotor1.set(-currentState.feederSpeed);

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
