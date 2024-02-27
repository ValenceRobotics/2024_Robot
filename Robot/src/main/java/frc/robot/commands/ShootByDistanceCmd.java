// package frc.robot.commands;

// import java.util.function.Supplier;

// import javax.xml.crypto.dsig.Transform;

// import org.photonvision.targeting.PhotonTrackedTarget;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Transform2d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.geometry.Translation3d;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.AutoConstants;
// import frc.robot.subsystems.AprilTagCamera;
// import frc.robot.subsystems.DriveSubsystem;
// import frc.robot.subsystems.PivotSubsystem;
// import frc.robot.subsystems.ShooterSubsystem;

// public class ShootByDistanceCmd extends Command {

//     private final DriveSubsystem dt;
//     private final AprilTagCamera cam;
//     private final PivotSubsystem arm;
//     private final ShooterSubsystem shooter;




//     public ShootByDistanceCmd(DriveSubsystem dt, AprilTagCamera cam, PivotSubsystem arm, ShooterSubsystem shooter) {
//         this.dt = dt;
//         this.cam = cam;
//         this.arm = arm;
//         this.shooter = shooter;
//         addRequirements(dt, cam, arm, shooter);
//     }

//     private static double sigmoid(double x) {
//     return 1 / (1 + Math.exp(-x));
//     }

//     @Override
//     public void initialize() {
//         Transform3d pose = cam.getTargetToCam();
//         double xDist = pose.getX();
//         double yDist = pose.getY();
//         double distance = (Math.sqrt((Math.pow(xDist, 2)) + (Math.pow(yDist, 3))));

//         //apply some function to distance
//         shooter.setShooterPower(sigmoid(distance));
//         arm.setPivotPosition(sigmoid(distance));

//     }

//   // Called every time the scheduler runs while the command is scheduled.
//     @Override
//     public void execute() {
        

//     }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }











// }