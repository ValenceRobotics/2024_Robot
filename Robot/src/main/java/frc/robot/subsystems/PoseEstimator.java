package frc.robot.subsystems;


import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PoseEstimator extends SubsystemBase {
    private AprilTagFieldLayout layout;
    private PhotonCamera cam;
    private Transform3d robotToCam;
    private PhotonPoseEstimator poseEstimator;
    private ArrayList<Pair<PhotonCamera, Transform3d>> camList;

    // private final Supplier<Rotation2d> rotationSupplier;
    // private final Supplier<SwerveModulePosition[]> modulePositionSupplier;
    // private final SwerveDrivePoseEstimator swervePoseEstimator;
  
    private Pose3d pose;
  
    private double x;
    private double y;
    private double z;
    private Rotation2d heading;


    public PoseEstimator() {

        cam = new PhotonCamera("USB_Camera");
        //to configure
        robotToCam = new Transform3d(new Translation3d(0.38, -0.15, 0.15), new Rotation3d(0,-30,0));
        //numbers reflect current
        camList = new ArrayList<>();
        camList.add(new Pair<PhotonCamera, Transform3d>(cam, robotToCam));

        // poseEstimator = new SwerveDrivePoseEstimator(
        // DrivetrainConstants.KINEMATICS,
        // rotationSupplier.get(),
        // modulePositionSupplier.get(),
        // new Pose2d(),);
    
       
        // try {
        //   layout = AprilTagFieldLayout.loadFromResource(String.valueOf(AprilTagFields.k2024Crescendo));
        // } catch (IOException e) {
        //   e.printStackTrace();
        // }s

        layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    
        // poseEstimator = new RobotPoseEstimator(layout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camList);
        poseEstimator = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cam, robotToCam);
    }

        @Override
        public void periodic() {
          Optional<EstimatedRobotPose> pose = poseEstimator.update();
        // public Optional<EstimatedRobotPose> getEstimatedGlobalPoe
          
          if(pose.isPresent()){
            Pose3d pose3d = pose.get().estimatedPose;
            this.pose = pose3d;
            Rotation3d rotation3d = pose3d.getRotation();
            x = pose3d.getX();
            y = pose3d.getY();
            z = pose3d.getZ();
            heading = rotation3d.toRotation2d();

            
            SmartDashboard.putNumber("PoseEstimator/Heading", heading.getDegrees());
            SmartDashboard.putNumber("PoseEstimator/X", x);
            SmartDashboard.putNumber("PoseEstimator/Y", y);
            SmartDashboard.putNumber("PoseEstimator/Z", z);
        }
        SmartDashboard.putNumber("PoseEstimator/test", 38);

    }
      
        public double getX(){
          return x;
        }
      
        public double getY(){
          return y;
        }
      
        public double getZ(){
          return z;
        }
      
        public Rotation2d getHeading(){
          return heading;
        }
      
        public Pose3d getPose(){
          return this.pose;
        }
      
    }



