package frc.robot.subsystems;


import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PoseEstimator extends SubsystemBase {
    private AprilTagFieldLayout layout;
    private PhotonCamera camFront;
    public Transform3d robotToCamFront;
    private PhotonPoseEstimator poseEstimatorFront;

    // private PhotonCamera camBack;
    // public Transform3d robotToCamBack;
    // private PhotonPoseEstimator poseEstimatorBack;
    // private ArrayList<Pair<PhotonCamera, Transform3d>> camList;

    // private final Supplier<Rotation2d> rotationSupplier;
    // private final Supplier<SwerveModulePosition[]> modulePositionSupplier;
    // private final SwerveDrivePoseEstimator swervePoseEstimator;
  
    private Pose3d pose;
  
    private double x;
    private double y;
    private double z;
    private Rotation2d heading;


    public PoseEstimator() {

        camFront = new PhotonCamera("visionCamFront");
        //camBack = new PhotonCamera("visionCamBack");
        //to configure
        robotToCamFront = new Transform3d(new Translation3d(0.0254, -0.324, Units.inchesToMeters(12)), new Rotation3d(0,-Units.degreesToRadians(19),0));
      //robotToCamFront = new Transform3d();

       // robotToCamBack = new Transform3d(new Translation3d(-0.1113,-0.32,0.435), new Rotation3d(0,Units.degreesToRadians(33),Units.degreesToRadians(180)));
        //numbers reflect current
        //camList = new ArrayList<>();
        //camList.add(new Pair<PhotonCamera, Transform3d>(camFront, robotToCamFront));

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
        poseEstimatorFront = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camFront, robotToCamFront);
        //poseEstimatorBack = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camBack, robotToCamBack);

    }

    public PhotonPoseEstimator getEstimatorFront(){
      return poseEstimatorFront;
    }

    // public PhotonPoseEstimator getEstimatorBack(){
    //   return poseEstimatorBack;
    // }

        @Override
        public void periodic() {
        //   Optional<EstimatedRobotPose> pose = poseEstimator.update();
        // // public Optional<EstimatedRobotPose> getEstimatedGlobalPoe
          
        //   if(pose.isPresent()){
            
        //     Pose3d pose3d = pose.get().estimatedPose;
      
        //     this.pose = pose3d;
        //     Rotation3d rotation3d = pose3d.getRotation();
        //     x = pose3d.getX();
        //     y = pose3d.getY();
        //     z = pose3d.getZ();
        //     heading = rotation3d.toRotation2d();

            
        //     SmartDashboard.putNumber("PoseEstimator/Heading", heading.getDegrees());
        //     SmartDashboard.putNumber("PoseEstimator/X", x);
        //     SmartDashboard.putNumber("PoseEstimator/Y", y);
        //     SmartDashboard.putNumber("PoseEstimator/Z", z);
        //       }

    }
      
    public PhotonPipelineResult getLatestFront() {
      return camFront.getLatestResult();
    }

    public double getDist() {
      return getLatestFront().getBestTarget().getBestCameraToTarget().getTranslation().getNorm();
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



