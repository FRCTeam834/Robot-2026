package frc.robot.subsystems.Vision;

import frc.robot.LimelightHelpers;

public class VisionIOLimelight implements VisionIO {
  // Variable to store the limelight name
  private final String name;

  // Constructor to set the camera name
  public VisionIOLimelight(String name) {
    this.name = name;
  }

  // Updates inputs object with new data
  public void updateInputs(VisionIOInputs inputs) {
    // Check if the limelight sees a target and save it
    inputs.hasTarget = LimelightHelpers.getTV(name);

    // Get pose estimate from MegaTag2
    // Grab even if hasTarget == false
    inputs.poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
  }
}