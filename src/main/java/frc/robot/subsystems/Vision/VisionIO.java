package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    // Position on field from camera
    public Pose3d[] poseEstimates = new Pose3d[] {};
    // Time when frame was captured
    public double[] timestampsSeconds = new double[] {};
    // Confidence/Error levels for each estimate
    public double[][] visionStdDevs = new double[][] {};
  }

  // Updates inputs from hardware or logs
  public default void updateInputs(VisionIOInputs inputs) {}
}
