// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonTargetSortMode;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilTagCamera extends SubsystemBase {

 
  PhotonCamera camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");

  private double pitch = 0;
  private double yaw = 0;
  private double skew = 0;
  private PhotonTrackedTarget target;

  public AprilTagCamera() {
  }

  @Override
  public void periodic() {
  //   var result = camera.getLatestResult();
  //   if(result.hasTargets()){
  //     var bestTarget = result.getBestTarget();
  //     target = bestTarget;
  //     pitch = bestTarget.getPitch();
  //     yaw = bestTarget.getYaw();
  //     skew = bestTarget.getSkew();
  //   }
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

}