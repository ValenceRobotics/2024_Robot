// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AprilTagCamera;

public class GetCameraPose extends Command {
  private AprilTagCamera cam;
 
  public GetCameraPose(AprilTagCamera cam) {
    this.cam = cam;
    addRequirements(cam);
  }

  @Override
  public void initialize() {}


  @Override
  public void execute() {
    var result = cam.getResult();
    if (result.hasTargets()) {
        PhotonTrackedTarget target = result.getBestTarget();
        Transform3d pose = target.getBestCameraToTarget();
        SmartDashboard.putNumber("Camera/Pose" ,pose.getX());
    } else{
      SmartDashboard.putNumber("Camera/Test", 4858);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}