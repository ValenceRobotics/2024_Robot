package frc.robot.commands;

import java.util.function.Supplier;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.AprilTagCamera;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class BareBonesAlignCmd extends Command {
    private final DriveSubsystem dt;
    private final AprilTagCamera cam;
    private final PivotSubsystem arm;
    private final ShooterSubsystem shooter;
    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
   //private static final TrapezoidProfile.Constraints O_CONSTRAINTS = AutoConstants.kThetaControllerConstraints;

    //to configure
    private static final Transform3d robotToCam = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0,0,0));
    
    private static final ProfiledPIDController xController = new ProfiledPIDController(.15, 0, 0, X_CONSTRAINTS);
    private static final ProfiledPIDController yController = new ProfiledPIDController(.15, 0, 0, Y_CONSTRAINTS);
    //private static final ProfiledPIDController oController = new ProfiledPIDController(.1, 0, 0, O_CONSTRAINTS);


    private final double pivotPosition = 0; //position in radians


    private static final int tagId = 10;

    private static final Transform3d goalPose = new Transform3d(new Translation3d(1, 0, 0), new Rotation3d(0, 0, Math.PI));

    public BareBonesAlignCmd(DriveSubsystem dt, AprilTagCamera cam, PivotSubsystem arm, ShooterSubsystem shooter) {
        this.dt = dt;
        this.cam = cam;
        this.arm = arm;
        this.shooter = shooter;

        xController.setTolerance(0.1);
        yController.setTolerance(0.1);
        addRequirements(dt, cam, arm, shooter);
    }

    @Override
    public void initialize() {

    }

  // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // var result = cam.getResult();
        // if (result.hasTargets()) {
        //     if (result.getBestTarget().getPoseAmbiguity() >= 0.2) {
        //     PhotonTrackedTarget target = result.getBestTarget();
        //     Transform3d pose = target.getBestCameraToTarget();
            Transform3d pose = cam.getTargetToCam();
            Transform3d robotToTag = robotToCam.plus(pose);

            double xSpeed = xController.calculate(robotToTag.getX(), goalPose.getX());
            double ySpeed = yController.calculate(robotToTag.getY(), goalPose.getY());
           // double oSpeed = oController.calculate(robotToTag.getRotation().getAngle(), goalPose.getRotation().getAngle());

           // SmartDashboard.putNumber("Tag Align/O", oSpeed);


            dt.drive(-xSpeed, -ySpeed, 0, false, true);
            arm.setPivotPosition(pivotPosition);
            shooter.setShooterPower(1);
      
        //    } else{
        //     SmartDashboard.putString("Drivetrain/Chase Tag", "Pose amb too low");
        //    }
        // } else{
        //     SmartDashboard.putString("Drivetrain/ Tag Chase", "No tag detected");
        // }

    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }











}
