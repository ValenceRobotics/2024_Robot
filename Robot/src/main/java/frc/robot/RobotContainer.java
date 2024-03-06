// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.ctre.phoenix6.unmanaged.Unmanaged;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.FieldMeasurements;
import frc.robot.Constants.IntakeState;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterState;
import frc.robot.commands.SetPivotPosition;
import frc.robot.commands.Climb.SetClimbLeftPower;
import frc.robot.commands.Climb.SetClimbRightPower;
import frc.robot.commands.Manipulator.SetMechanismState;
import frc.robot.commands.drive.SetSlowMode;
import frc.robot.commands.drive.SwerveDrive;
import frc.robot.commands.drive.Align.AlignToTarget;
import frc.robot.commands.drive.Align.DriveToTarget;
import frc.robot.subsystems.AprilTagCamera;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ClimberSubsystem2;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeFeederSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.ShooterSubsystem;
 
/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public static final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public static final ShooterSubsystem m_Shooter = new ShooterSubsystem();
  public static final PoseEstimator m_PoseEstimator = new PoseEstimator();
  public static final IntakeFeederSubsystem m_IntakeFeederSubsystem = new IntakeFeederSubsystem();
  private final AprilTagCamera m_AprilTagCamera = new AprilTagCamera();
  private final PivotSubsystem m_PivotSubsystem = new PivotSubsystem();
  private final ClimberSubsystem m_Climber = new ClimberSubsystem();
  private final ClimberSubsystem2 m_Climber2 = new ClimberSubsystem2();
  public static boolean isRed = false;
  public static Command currentAlignPath = new InstantCommand(); 
  // Autonomous chooser
  private final SendableChooser<Command> m_autoChooser;

  // The driver's controller
  //XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    Joystick m_driverController = new Joystick(0);

    CommandXboxController m_OperatorController = new CommandXboxController(1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    CameraServer.startAutomaticCapture();

    DataLogManager.stop();
    
    Unmanaged.setPhoenixDiagnosticsStartTime(-1);
    // Configure default commands
    m_robotDrive.setDefaultCommand(
            new SwerveDrive(m_robotDrive,
                    () -> -MathUtil.applyDeadband(m_driverController.getRawAxis(1),
                            OIConstants.kDriveDeadband),
                    () -> -MathUtil.applyDeadband(m_driverController.getRawAxis(0),
                            OIConstants.kDriveDeadband),
                    () -> -0.75*MathUtil.applyDeadband(m_driverController.getRawAxis(2),
                            OIConstants.kDriveDeadband)));

    //m_PivotSubsystem.setDefaultCommand(new OpenLoopPivot(m_PivotSubsystem, () -> (-0.3 * m_OperatorController.getRawAxis(1))));
    
  
    //Register Named Commands 
    NamedCommands.registerCommand("shootUpClose", (new SetPivotPosition(m_PivotSubsystem, PivotConstants.kSubwooferSideShot).withTimeout(1) )
                                             .andThen(new SetMechanismState( ShooterState.SHOOTING))
                                            .andThen(new WaitCommand(1))
                                            .andThen(new SetMechanismState(IntakeState.SHOOTING))
                                            .andThen(new WaitCommand(0.5))
                                              .andThen(new SetMechanismState(IntakeState.STOPPED, ShooterState.STOPPED))
                                              .andThen(new SetPivotPosition(m_PivotSubsystem, PivotConstants.kHomePosition).withTimeout(1))
                                            

    
    );


    NamedCommands.registerCommand("intakeStart", new SetPivotPosition(m_PivotSubsystem, PivotConstants.kIntakePosition).alongWith(new SetMechanismState(IntakeState.INTAKING, ShooterState.INTAKING)));



    NamedCommands.registerCommand("intakeStop", new SetMechanismState(IntakeState.STOPPED, ShooterState.STOPPED).andThen(new SetPivotPosition(m_PivotSubsystem, PivotConstants.kHomePosition)));

    m_autoChooser = AutoBuilder.buildAutoChooser();
    Shuffleboard.getTab("Dashboard").add("Auto Chooser", m_autoChooser);
    SendableChooser<Boolean> debugMode = new SendableChooser<>();
    debugMode.addOption("Enabled", true);
    debugMode.setDefaultOption("Disabled", false);
    debugMode.onChange(x-> Constants.DebugConstants.kDebugMode = x);
    Shuffleboard.getTab("Dashboard").add("Debug Mode", debugMode);





    //NamedCommands.registerCommand"shootAmpt", //amp command here )
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, 1)
        .whileTrue(new SetMechanismState(IntakeState.SHOOTING)).onFalse(new SetMechanismState(IntakeState.STOPPED).andThen(new WaitCommand(0.5).andThen(new SetPivotPosition(m_PivotSubsystem, PivotConstants.kHomePosition))));

    new JoystickButton(m_driverController, 12).onTrue(new InstantCommand(m_robotDrive::resetGyro));

    new JoystickButton(m_driverController,2)
    .whileTrue(new SetSlowMode(m_robotDrive, true))
    .whileFalse(new SetSlowMode(m_robotDrive, false));

    new JoystickButton(m_driverController, 5)
      .whileTrue(new SetClimbLeftPower(m_Climber, 1));
    
    new JoystickButton(m_driverController, 9)
      .whileTrue(new SetClimbLeftPower(m_Climber, -1));

    new JoystickButton(m_driverController, 6)
      .whileTrue(new SetClimbRightPower(m_Climber2, 1));

    new JoystickButton(m_driverController, 10)
      .whileTrue(new SetClimbRightPower(m_Climber2, -1));

    m_OperatorController.rightTrigger().whileTrue(new SetMechanismState(ShooterState.SHOOTING)).onFalse(new SetMechanismState(ShooterState.STOPPED));
    m_OperatorController.leftTrigger().whileTrue(new SetMechanismState(ShooterState.AMP)).onFalse(new SetMechanismState(ShooterState.STOPPED));

    new JoystickButton(m_driverController, 4).whileTrue(new SetMechanismState(IntakeState.INTAKING, ShooterState.INTAKING).alongWith(new SetPivotPosition(m_PivotSubsystem, PivotConstants.kIntakePosition)).alongWith(new SetSlowMode(m_robotDrive, true))).onFalse(new SetMechanismState(IntakeState.STOPPED, ShooterState.STOPPED).andThen(new SetPivotPosition(m_PivotSubsystem, PivotConstants.kHomePosition).alongWith(new SetSlowMode(m_robotDrive, false))));
    m_OperatorController.leftBumper().whileTrue(new SetMechanismState(IntakeState.OUTTAKING, ShooterState.OUTTAKING)).onFalse(new SetMechanismState(IntakeState.STOPPED, ShooterState.STOPPED));

    m_OperatorController.button(8).whileTrue(Commands.run(() -> m_PivotSubsystem.setGoal((2*Math.PI/3)*Math.abs(m_OperatorController.getRawAxis(1)))));
    


    

    // m_OperatorController.a().whileTrue(m_Shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // m_OperatorController.b().whileTrue(m_Shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));


    // m_OperatorController.x().whileTrue(m_Shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // m_OperatorController.y().whileTrue(m_Shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    m_OperatorController.x().onTrue(new SetPivotPosition(m_PivotSubsystem, PivotConstants.kAmpPosition));
    m_OperatorController.a().onTrue(new SetPivotPosition(m_PivotSubsystem, PivotConstants.kIntakePosition));
    // SmartDashboard.putNumber("test pivot loc", PivotConstants.kHomePosition);
    // m_OperatorController.y().onTrue(new SetPivotPosition(m_PivotSubsystem, ()->SmartDashboard.getNumber("test pivot loc", PivotConstants.kHomePosition)));
    m_OperatorController.y().whileTrue(new AlignToTarget(m_robotDrive));
    // test vision/rotation
    // test pivot down on intake
    // check dashboard/debugmode
    // 
    // m_OperatorController.b().whileTrue(new DriveToTarget()).onFalse(new InstantCommand(()->currentAlignPath.cancel()));
    // ALIGN CODE ^^^^

    m_OperatorController.rightStick().whileTrue(new SetClimbRightPower(m_Climber2, 1).alongWith(new SetClimbLeftPower(m_Climber, 1)));
    
    
    
    //.andTh  
    // en(()-> currentAlignPath));

    m_OperatorController.start().onTrue(new SetPivotPosition(m_PivotSubsystem, true));
    
    m_OperatorController.povUp().onTrue(new SetPivotPosition(m_PivotSubsystem, PivotConstants.kSubwooferShot));
    m_OperatorController.povDown().onTrue(new SetPivotPosition(m_PivotSubsystem, PivotConstants.kAmpPosition));
    m_OperatorController.povRight().onTrue(new SetPivotPosition(m_PivotSubsystem, PivotConstants.kHomePosition));


   




  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // private void configureAutoChooser() {
  //       // Configure autonomous chooser with different auto paths
  //       m_autoChooser.addOption("Auto Path 1", new PathPlannerAuto("Auto"));
  //       m_autoChooser.addOption("Auto Path 2", new PathPlannerAuto("New Auto"));
  //       // Add more paths as needed

  //       // Put the chooser on the dashboard
  //       SmartDashboard.putData("Auto Mode", m_autoChooser);
  //   }
  public Command getAutonomousCommand() {
    // // Create config for trajectory
    // TrajectoryConfig config = new TrajectoryConfig(
    //     AutoConstants.kMaxSpeedMetersPerSecond,
    //     AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //     // Add kinematics to ensure max speed is actually obeyed
    //     .setKinematics(DriveConstants.kDriveKinematics);

    // // An example trajectory to follow. All units in meters.
    // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    //     // Start at the origin facing the +X direction
    //     new Pose2d(0, 0, new Rotation2d(0)),
    //     // Pass through these two interior waypoints, making an 's' curve path
    //     List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    //     // End 3 meters straight ahead of where we started, facing forward
    //     new Pose2d(3, 0, new Rotation2d(0)),
    //     config);

    // var thetaController = new ProfiledPIDController(
    //     AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    //     exampleTrajectory,
    //     m_robotDrive::getPose, // Functional interface to feed supplier
    //     DriveConstants.kDriveKinematics,

    //     // Position controllers
    //     new PIDController(AutoConstants.kPXController, 0, 0),
    //     new PIDController(AutoConstants.kPYController, 0, 0),
    //     thetaController,
    //     m_robotDrive::setModuleStates,
    //     m_robotDrive);

    // // Reset odometry to the starting pose of the trajectory.
    // m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // // Run path following command, then stop at the end.
    // return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
    return m_autoChooser.getSelected();
  }
}
