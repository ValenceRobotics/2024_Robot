// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.DebugConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldMeasurements;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.IOException;
import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

public class DriveSubsystem extends SubsystemBase {

  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  //private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();
  private final Pigeon2 m_gyro = new Pigeon2(5);

  public final Field2d m_field = new Field2d();
  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  private boolean fieldRelative;
  private boolean slowMode = false;
    private AprilTagFieldLayout aprilTagFieldLayout = null;


  public DriveSubsystem(boolean fieldRelative) {
    this.fieldRelative = fieldRelative;

       AprilTagFieldLayout layout;
 
      layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
      var allianceB = DriverStation.getAlliance();
      var alliance = allianceB.isPresent() ? allianceB.get() : Alliance.Blue;
      layout.setOrigin(alliance == Alliance.Blue ?
          OriginPosition.kBlueAllianceWallRightSide : OriginPosition.kRedAllianceWallRightSide);
   
    this.aprilTagFieldLayout = layout;
  }

  public boolean getFieldRelative() {
    return fieldRelative;
  }

  public void setFieldRelative(boolean fieldRelative) {
    this.fieldRelative = fieldRelative;
  }

  public boolean getSlowMode() {
    return slowMode;
  }

  public double calcPivotAngle() {
    double x = this.getDistToTarget();
    return (-8.63e-4*Math.pow(x, 4) + 3.01e-3*Math.pow(x,3) + 0.052*Math.pow(x,2) -0.388*x + 1.59);
    // double x = this.getDistToTarget();
    // return (0.001857 * Math.pow(x, 9) - 0.04014 * Math.pow(x, 8) + 0.307788 * Math.pow(x, 7) - 
    // 0.6034 * Math.pow(x, 6) - 5.03147503 * Math.pow(x, 5) + 40.8601621 * Math.pow(x, 4) - 127.70156 * Math.pow(x, 3) + 
    // 210.01561 * Math.pow(x, 2) - 176.147701 * Math.pow(x, 1) + 59.196585 * Math.pow(x, 0));
  }

  public void setSlowMode(boolean slowMode) {
    this.slowMode = slowMode;
  }

  // Odometry class for tracking robot pose
  SwerveDrivePoseEstimator m_odometry = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      m_gyro.getRotation2d(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      }, new Pose2d());

  public void resetGyro() {
    m_gyro.reset();
  }

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    
     AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    //TO CONFIGURE
                    new PIDConstants(6, 0, 0), // Translation PID constants Neoprene: 8, 0, 0
                    new PIDConstants(5, 0.0, 0.0), // Rotation PID constants
    5.7, // Max module speed, in m/s
                    Units.inchesToMeters(18.7383297), // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );

    Pathfinding.setPathfinder(new LocalADStar());
    //align code 14.66 7.57 -90

    SmartDashboard.putData("Field", m_field);

            // Logging callback for current robot pose
            PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
              // Do whatever you want with the pose here
              m_field.setRobotPose(pose);
          });
  
          // Logging callback for target robot pose
          PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
              // Do whatever you want with the pose here
              m_field.getObject("target pose").setPose(pose);
          });
  
          // Logging callback for the active path, this is sent as a list of poses
          PathPlannerLogging.setLogActivePathCallback((poses) -> {
              // Do whatever you want with the poses here
              m_field.getObject("path").setPoses(poses);
          });

  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block

  //   var bruh = RobotContainer.m_PoseEstimator.getEstimatorFront();
  //  // var rearPoseEstimator = RobotContainer.m_PoseEstimator.getEstimatorBack();
  //   var visionPoseFront = bruh.update();
   // var visionPoseBack = rearPoseEstimator.update();

   RobotContainer.m_PoseEstimator.updatePoseEstimator();
    var visionPoseFront = RobotContainer.m_PoseEstimator.getVisionPoseFront();

    if(visionPoseFront!=null && visionPoseFront.isPresent() && RobotContainer.m_PoseEstimator.canSeeTarget()) {
      var huh = visionPoseFront.get();
      Pose3d pose3d = huh.estimatedPose;

      var savedDist = RobotContainer.m_PoseEstimator.getDist();

      if(DebugConstants.kDebugMode) {
        SmartDashboard.putNumber("std-dev used distance", savedDist);
      }

      if(savedDist < 6.7) {
        m_odometry.addVisionMeasurement(pose3d.toPose2d(), huh.timestampSeconds, VecBuilder.fill(savedDist/2, savedDist/2, Units.degreesToRadians(20)));

      }

      SmartDashboard.putString("pose from vision front ", pose3d.toPose2d().toString());
    }

    // if (SmartDashboard.getNumber("Tags Detected",0) > 0) {
    //   Transform3d robotToCam = new Transform3d(new Translation3d(0.0157, -0.3708, 0.4064), new Rotation3d());

    //   double poseX = SmartDashboard.getNumber("vision_x",0);
    //   double poseY = SmartDashboard.getNumber("vision_y",0);
    //   double poseZ = SmartDashboard.getNumber("vision_z",0);
    //   double posePitch = SmartDashboard.getNumber("vision_pitch",0);
    //   double poseRoll = SmartDashboard.getNumber("vision_Roll",0);
    //   double poseYaw = SmartDashboard.getNumber("vision_yaw",0);
    //   poseYaw -= Units.degreesToRadians(270);

    //   double xOffset = -0.0157;
    //   double yOffset = 0.3708;
    //   double zOffset = -0.4064;

    //   double robotX = poseX+(xOffset*Math.cos(poseYaw)-yOffset*Math.sin(poseYaw));
    //   double robotY = poseY+(xOffset*Math.sin(poseYaw)+yOffset*Math.cos(poseYaw));
    //   double robotZ = poseZ+zOffset;

    //   Pose3d oakPose = new Pose3d(poseX, poseY, poseZ, new Rotation3d(poseRoll, posePitch, poseYaw));

    //   //oakPose = oakPose.plus(robotToCam.inverse());


    //   SmartDashboard.putString("Oak pose",  oakPose.toPose2d().toString());

    // }else{
    //   SmartDashboard.putString("Oak pose", (new Pose2d()).toString());
    // }


    // if (visionPoseBack.isPresent()) {
    //   var huh2 = visionPoseBack.get();
    //   Pose3d pose3dback = huh2.estimatedPose;
    //   m_odometry.addVisionMeasurement(pose3dback.toPose2d(), huh2.timestampSeconds);
    //   SmartDashboard.putString("pose from vision back", pose3dback.toPose2d().toString());
    // }


      // Update pose estimator with the best visible target
    // var pipelineResult = RobotContainer.m_PoseEstimator.getLatest();
    // var resultTimestamp = pipelineResult.getTimestampSeconds();
    // if (pipelineResult.hasTargets()) {
    //   var target = pipelineResult.getBestTarget();
    //   var fiducialId = target.getFiducialId();
    //   // Get the tag pose from field layout - consider that the layout will be null if it failed to load
    //   Optional<Pose3d> tagPose = aprilTagFieldLayout == null ? Optional.empty() : aprilTagFieldLayout.getTagPose(fiducialId);
    //   if (target.getPoseAmbiguity() <= .2 && fiducialId >= 0 && tagPose.isPresent()) {
    //     var targetPose = tagPose.get();
    //     Transform3d camToTarget = target.getBestCameraToTarget();
    //     Pose3d camPose = targetPose.transformBy(camToTarget.inverse());
 
    //     var visionMeasurement = camPose.transformBy(RobotContainer.m_PoseEstimator.robotToCam.inverse());
    //     m_odometry.addVisionMeasurement(visionMeasurement.toPose2d(), resultTimestamp);
    //   }
    // }

        m_odometry.update(
      m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    Pose2d pose = m_odometry.getEstimatedPosition();

    if (DebugConstants.kDebugMode) {

    SmartDashboard.putString("Pose2D Pose: ", pose.toString());


    SmartDashboard.putNumber("Drivetrain/X", pose.getX());
    SmartDashboard.putNumber("Drivetrain/Y", pose.getY());
    SmartDashboard.putNumber("Drivetrain/Heading", pose.getRotation().getDegrees());

    SmartDashboard.putNumber("Drivetrain/Gyro", m_gyro.getRotation2d().getDegrees());

    double loggingState[] = {
    Math.IEEEremainder(m_frontLeft.getState().angle.getDegrees(),360)+180,
    Math.IEEEremainder(m_frontLeft.getDesiredState().angle.getDegrees(),360),
     //m_frontRight.getState().speedMetersPerSecond,
   //m_rearLeft.getState().speedMetersPerSecond,
    //m_rearRight.getState().speedMetersPerSecond,
    };


    double commandedState[] = {
    m_frontLeft.getDesiredState().speedMetersPerSecond
   // m_frontRight.getDesiredState().speedMetersPerSecond,
   // m_rearLeft.getDesiredState().speedMetersPerSecond,
   // m_rearRight.getDesiredState().speedMetersPerSecond,
    };

    SmartDashboard.putNumberArray("SwerveModuleStates", loggingState);
   // SmartDashboard.putNumberArray("SwerveModuleDESIREDStates", commandedState);






    SmartDashboard.putNumber("translational dist to target",this.getDistToTarget() );

    }


    // m_field.setRobotPose(m_odometry.getEstimatedPosition());

    

  }

  public double getDistToTarget(){
        var alliance = DriverStation.getAlliance();
    boolean isRed = false; 
    if (alliance.isPresent()) {
      isRed = alliance.get() == DriverStation.Alliance.Red;
    }
    Translation2d target;
    if(!isRed){
      target = FieldMeasurements.blueTarget.getTranslation();
    } else{
      target = FieldMeasurements.redTarget.getTranslation();
    }
    return this.getPose().getTranslation().getDistance(target);


  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {

    var iable = pose.getRotation();
    iable = RobotContainer.isRed ? iable : iable.unaryMinus();

   m_gyro.setYaw(iable.getDegrees());

    m_odometry.resetPosition(
      iable,
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);

        
  }

   
  


  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    
    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }
      

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;
      
      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);


    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, m_gyro.getRotation2d()
            )
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);// front left
    m_frontRight.setDesiredState(swerveModuleStates[1]);//front right
    m_rearLeft.setDesiredState(swerveModuleStates[2]); //back left
    m_rearRight.setDesiredState(swerveModuleStates[3]); //back right
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }
  
  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    states[0] = m_frontLeft.getState();
    states[1] = m_frontRight.getState();
    states[2] = m_rearLeft.getState();
    states[3] = m_rearRight.getState();
    return states;
  }


  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return this.getPose().getRotation().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

    public ChassisSpeeds getRobotRelativeSpeeds() {
    return ChassisSpeeds.fromFieldRelativeSpeeds(Constants.DriveConstants.kDriveKinematics.toChassisSpeeds(getStates()),
        m_gyro.getRotation2d());
  }


  public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
    SwerveModuleState[] newStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(newStates, DriveConstants.kMaxSpeedMetersPerSecond);
    setModuleStates(newStates);
    
  }
}
