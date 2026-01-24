package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.LimelightHelpers;


public class VisionIOLimelight implements VisionIO {
  // Name of Limelight instance
  private final String name;

  public VisionIOLimelight(String name) {
    this.name = name;
  }
  // Update camera inputs
  public void updateInputs(VisionIOInputs inputs) {
    // Grab latest camera results
    var results = LimelightHelpers.getLatestResults(name);
    // If the camera sees a target
    if (results.valid) {
      inputs.poseEstimates = new Pose3d[] {results.getBotPose3d_wpiBlue()};
      // Calculate exact time of the photo 
      double timestamp = Timer.getFPGATimestamp() - (results.latency_capture / 1000.0) - (results.latency_pipeline / 1000.0); // subtracts lag
      // Save the timing
      inputs.timestampsSeconds = new double[] {timestamp};
    } else {
      // Clear data if nothing is seen
      inputs.poseEstimates = new Pose3d[] {};
      inputs.timestampsSeconds = new double[] {};
    }
  }
}
