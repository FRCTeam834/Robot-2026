package frc.robot.subsystems.Vision;

import frc.robot.LimelightHelpers;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  // tells AdvantageKit to log this class
  @AutoLog
  public static class VisionIOInputs {
    // True if camera sees tag
    public boolean hasTarget = false;
    
    // Calculated position of robot
    public LimelightHelpers.PoseEstimate poseEstimate = new LimelightHelpers.PoseEstimate();
  }

  // Method to update the inputs variable
  public default void updateInputs(VisionIOInputs inputs) {}
}