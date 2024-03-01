// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DebugConstants;

public class AprilTagCamera extends SubsystemBase {

 
  PhotonCamera camera = new PhotonCamera("USB_Camera");

  private double pitch = 0;
  private double yaw = 0;
  private double skew = 0;
  private PhotonTrackedTarget target;

  public AprilTagCamera() {
  }

  @Override
  public void periodic() {
    if (DebugConstants.kDebugMode) {
          var result = camera.getLatestResult();
      SmartDashboard.putBoolean("Camera/HasTarget", result.hasTargets());
      if(result.hasTargets()){
      var bestTarget = result.getBestTarget();
      SmartDashboard.putNumber("Camera/Tag ID", bestTarget.getFiducialId());
      SmartDashboard.putNumber("Camera/Pose Ambuguity", bestTarget.getPoseAmbiguity());
      target = bestTarget;
      Transform3d camToTag = bestTarget.getBestCameraToTarget();
      SmartDashboard.putNumber("Camera/X", camToTag.getX());
      SmartDashboard.putNumber("Camera/Y", camToTag.getY());
      SmartDashboard.putNumber("Camera/Z", camToTag.getZ());
      pitch = bestTarget.getPitch();
      yaw = bestTarget.getYaw();
      skew = bestTarget.getSkew();

    }

    }
   }

  public double getPitch() {
    return pitch;
  }

  public double getYaw() {
    return yaw;
  }

  public double getSkew() {
    return skew;
  }

  public PhotonPipelineResult getResult(){
    var result = camera.getLatestResult();
    return result;

  }

  public Transform3d getTargetToCam() {
    return target.getBestCameraToTarget();
  }

}