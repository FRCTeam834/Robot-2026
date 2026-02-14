package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  // tells AdvantageKit to log this class
  @AutoLog
  public static class VisionIOInputs {
    public boolean hasTarget = false;
    public int tagCount = 0;
    public Pose2d pose = new Pose2d();
    public double timestampSeconds = 0.0;
  }

  // Method to update the inputs variable
  public default void updateInputs(VisionIOInputs inputs) {}
}
