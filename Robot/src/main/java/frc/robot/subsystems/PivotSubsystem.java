// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DebugConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.PivotConstants;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.MutableMeasure.mutable;


import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PivotSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

    private final CANSparkMax pivotMotor1;
    private final CANSparkMax pivotMotor2;
    private final ProfiledPIDController pivotController;
    private final ArmFeedforward pivotFeedforward;
    private final AbsoluteEncoder absEncoder;
    private final double pivotVelocity;
    private final PIDController pivotPIDController = new PIDController(.3, 0,0); //0.6, 0.0015, 0.002
    private double goal = PivotConstants.kHomePosition;
    private double gravConst = 0.026;
    //abs encoder
    //possible feedforward?

    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    private final MutableMeasure<Angle> m_angularPosition = mutable(Radians.of(0));
    private final MutableMeasure<Velocity<Angle>> m_angularVelocity = mutable(RadiansPerSecond.of(0));

  private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(Volts.per(Second).of(0.5), Volts.of(2.5), Second.of(10)),
      new SysIdRoutine.Mechanism(
      (Measure<Voltage> volts) -> {
        this.setPivotPower(volts.in(Volts)/RobotController.getBatteryVoltage());
      },
      log -> {
        log.motor("pivotMotors")
          .voltage(
            m_appliedVoltage.mut_replace(this.getPivotVoltage(), Volts))
            .angularPosition(m_angularPosition.mut_replace(this.getPivotPosition(), Radians))
            .angularVelocity(m_angularVelocity.mut_replace(this.getPivotVelocity(), RadiansPerSecond));
      },
      this
    ));

  public PivotSubsystem() {
    pivotMotor1 = createPivotController(PivotConstants.pivotMotor1Id, false);
    pivotMotor2 = createPivotController(PivotConstants.pivotMotor2Id, true);

    pivotController = new ProfiledPIDController(0.5, 0, 0, PivotConstants.kPivotControllerConstraints);
    pivotFeedforward = new ArmFeedforward(0, 0, 0);
    absEncoder = pivotMotor2.getAbsoluteEncoder(Type.kDutyCycle);
    absEncoder.setInverted(true);
    pivotPIDController.enableContinuousInput(0, 2*Math.PI);
    pivotPIDController.setTolerance(Units.degreesToRadians(1));
    SmartDashboard.putNumber("Pivot/grav const", this.gravConst);

    SmartDashboard.putData("Pivot/PID", pivotPIDController);
    // SmartDashboard.putNumber("Pivot P", pivotP);
    // SmartDashboard.putNumber("Pivot I", pivotI);
    // SmartDashboard.putNumber("Pivot D", pivotD);
    // pivotPIDController.setP(SmartDashboard.getNumber("Pivot P", 0));
    // pivotPIDController.setI(SmartDashboard.getNumber("Pivot I", 0));
    // pivotPIDController.setD(SmartDashboard.getNumber("Pivot D", 0));

    // Shuffleboard.getTab("Pivot").add("pid test", pivotPIDController);

    //absEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
    //absEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);
    //absEncoder.setPositionOffset(PivotConstants.positionOffset);


    pivotVelocity = 0.5;

  }


  //set position method
  //get position method
  
  // public void setPivotPosition(double position) {
  //   //double pivotFeed = pivotFeedforward.calculate(position, pivotVelocity);
  //   double pidFeed = pivotController.calculate(getPivotPosition(), position);
  //   setPivotPower(-pidFeed);
  // }

  public double getPivotPosition() {
    return absEncoder.getPosition() * 2* Math.PI;
  }

  public double getPivotVoltage() {
    return pivotMotor1.get() * RobotController.getBatteryVoltage();
  }

  public double getPivotVelocity() {
    return absEncoder.getVelocity() * 2* Math.PI;
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

  public void setGoal(double newGoal) {
    this.goal = newGoal;
  }

  public double getGoal() {
    return this.goal;
  }

  public boolean atGoal() {
    return pivotPIDController.atSetpoint();
  }



  private CANSparkMax createPivotController(int port, boolean isInverted) {
    CANSparkMax controller = new CANSparkMax(port, MotorType.kBrushless);
    controller.restoreFactoryDefaults();
    

    controller.setIdleMode(CANSparkMax.IdleMode.kBrake);
    controller.setSmartCurrentLimit(40);

    controller.setInverted(isInverted);

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

    double output = pivotPIDController.calculate(getPivotPosition(), this.goal);
    if (DebugConstants.kDebugMode) {
    SmartDashboard.putNumber("Pivot/Position", getPivotPosition());
    SmartDashboard.putNumber("Pivot/Test", 39585);

    SmartDashboard.putNumber("Pivot/Current 1", pivotMotor1.getOutputCurrent());
    SmartDashboard.putNumber("Pivot/Current 2", pivotMotor2.getOutputCurrent());
    SmartDashboard.putNumber("Pivot/Output power", output);
    SmartDashboard.putBoolean("Pivot/atsetpoint", pivotPIDController.atSetpoint());
    SmartDashboard.putNumber("Pivot/p", pivotPIDController.getP());
    SmartDashboard.putNumber("Pivot/d", pivotPIDController.getD());
    SmartDashboard.putNumber("Pivot/i", pivotPIDController.getI());
    SmartDashboard.putNumber("Pivot/Goal", this.goal);



    }

    if (this.goal == PivotConstants.kIntakePosition) {
      gravConst = 0;
    } else {
      gravConst = 0.026;
    }

    setPivotPower((output)+(this.gravConst*Math.cos(getPivotPosition()-0.1)));
  
  }
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
