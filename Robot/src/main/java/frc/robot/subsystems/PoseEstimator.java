package frc.robot.subsystems;


import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class PoseEstimator extends SubsystemBase {
    //private AprilTagFieldLayout layout;
    private PhotonCamera cam;
    // private Transform3d robotToCam;
    // private PhotonPoseEstimator poseEstimator;
    // private ArrayList<Pair<PhotonCamera, Transform3d>> camList;

    // private final Supplier<Rotation2d> rotationSupplier;
    // private final Supplier<SwerveModulePosition[]> modulePositionSupplier;
    // private final SwerveDrivePoseEstimator swervePoseEstimator;
  
    // private Pose3d pose;
  
    // private double x;
    // private double y;
    // private double z;
    // private Rotation2d heading;


    public PoseEstimator() {

        cam = new PhotonCamera("Microsoft_LifeCam_HD-3000");
        //to configure
        //robotToCam = new Transform3d(new Translation3d(0, 0.0, 0), new Rotation3d(0,0,0));
        // camList = new ArrayList<>();
        // camList.add(new Pair<PhotonCamera, Transform3d>(cam, robotToCam));

        // poseEstimator = new SwerveDrivePoseEstimator(
        // DrivetrainConstants.KINEMATICS,
        // rotationSupplier.get(),
        // modulePositionSupplier.get(),
        // new Pose2d(),);
    
       
        // try {
        //   layout = AprilTagFieldLayout.loadFromResource(String.valueOf(AprilTagFields.kDefaultField));
        // } catch (IOException e) {
        //   e.printStackTrace();
        // }
    
        // poseEstimator = new RobotPoseEstimator(layout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camList);
        //poseEstimator = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cam, robotToCam);
    }

        @Override
        public void periodic() {
        //   Optional<EstimatedRobotPose> pose = poseEstimator.update();
          
        //   if(pose.isPresent()){
        //     Pose3d pose3d = pose.get().estimatedPose;
        //     this.pose = pose3d;
        //     Rotation3d rotation3d = pose3d.getRotation();
        //     x = pose3d.getX();
        //     y = pose3d.getY();
        //     z = pose3d.getZ();
        //     heading = rotation3d.toRotation2d();

        var result = cam.getLatestResult();
        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            Transform3d pose = target.getBestCameraToTarget();
            
        
      
            SmartDashboard.putNumber("PoseEstimator/X", pose.getX());
            SmartDashboard.putNumber("PoseEstimator/Y", pose.getY());
            SmartDashboard.putNumber("PoseEstimator/Z", pose.getZ());
            SmartDashboard.putNumber("PoseEstimator/test", 38);
            //SmartDashboard.putNumber("Drivetrain/Heading", heading.getDegrees());
          }
        }
        
      
        // public double getX(){
        //   return x;
        // }
      
        // public double getY(){
        //   return y;
        // }
      
        // public double getZ(){
        //   return z;
        // }
      
        // public Rotation2d getHeading(){
        //   return heading;
        // }
      
        // public Pose3d getPose(){
        //   return this.pose;
        // }
      
    }
    


