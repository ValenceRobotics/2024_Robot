// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;



/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class DebugConstants {
    public static boolean kDebugMode = false;
  }
  

  public static final class FieldMeasurements {
   public static Pose2d blueTarget = new Pose2d(0.5, 5.53, new Rotation2d(Units.degreesToRadians(180)));
   public static Pose2d redTarget =  new Pose2d(16.1, 5.53, new Rotation2d(Units.degreesToRadians(0)));

   public static Pose2d blueLineupSub = new Pose2d(1.36, 5.53, new Rotation2d(Units.degreesToRadians(180)));
   public static Pose2d redLineupSub =  new Pose2d(15.18, 5.53, new Rotation2d(Units.degreesToRadians(0)));

   public static Pose2d blueLineupAmp = new Pose2d(1.81, 7.65, new Rotation2d(Units.degreesToRadians(90)));
   public static Pose2d redLineupAmp =  new Pose2d(14.7, 7.65, new Rotation2d(Units.degreesToRadians(90)));



  }

  public static class ShooterConstants {
    public static final int shooterMotortopId = 21;
    public static final int shooterMotorbottomId = 26;
    public static final double shooterPower = 0.3;
    public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;
    public static final CurrentLimitsConfigs CURRENT_LIMIT = new CurrentLimitsConfigs().withStatorCurrentLimit(80);
    public static final int vortexRPM = 6784;
   }

  public static final class ClimberConstants {
    public static final int climbMotor1Id = 23;
    public static final int climbMotor2Id = 24;

    public static final double kMaxExtendPosition = 10;
    public static final double kMinExtendPosition = 1;
   }

  public static class IntakeFeederConstants {
    public static final int intakeMotorId = 25;
    public static final int feederMotor1Id = 18;
  }

  public static enum IntakeState {
    STOPPED(0,0), SHOOTING(0,-1), INTAKING(0.8,0.6), OUTTAKING(-0.3,-0.3);
    public final double intakeSpeed;
    public final double feederSpeed;
    private IntakeState(double intake, double feeder){
      this.intakeSpeed = intake;
      this.feederSpeed = feeder;
    }
  }
  public static enum ShooterState {
    STOPPED(0,0), SHOOTING(0.9,1), INTAKING(-0.6,-0.6),OUTTAKING(0.525,0.325), AMP(0.125,0.15), TRAP(0.375, 0.475);

    public final double lowSpeed;
    public final double highSpeed;
    private ShooterState(double shooty, double shooty2){
      this.lowSpeed = shooty;
      this.highSpeed = shooty2;
    }


  }

  public static class PivotConstants {
    public static final int pivotMotor1Id = 20;
    public static final int pivotMotor2Id = 22;

    public static final TrapezoidProfile.Constraints kPivotControllerConstraints = new TrapezoidProfile.Constraints(
      0.5, 0);

    public static final double kHomePosition = 0.3;
    public static final double kAmpPosition = 2.01;
    public static final double kIntakePosition = 0.02;
    public static final double kPodiumPosition = 0.9;

    public static final double kSubwooferShot = 1.17;//- Units.degreesToRadians(7.5);

    public static final double kSubwooferSideShot = 1.17- Units.degreesToRadians(7.5);



    
  }
   
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 5.7;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 10; // radians per second
    public static final double kMagnitudeSlewRate = 10; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 10; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2 + Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0 + Math.PI / 2;
    public static final double kBackLeftChassisAngularOffset = Math.PI + Math.PI / 2;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2 + Math.PI / 2;

    // SPARK MAX CAN IDs

    public static final int kFrontLeftDrivingCanId = 13;
    public static final int kRearLeftDrivingCanId = 17;
    public static final int kFrontRightDrivingCanId = 11;
    public static final int kRearRightDrivingCanId = 15;

    public static final int kFrontLeftTurningCanId = 12;
    public static final int kRearLeftTurningCanId = 16;
    public static final int kFrontRightTurningCanId = 10;
    public static final int kRearRightTurningCanId = 14;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 6784;
  }
}
