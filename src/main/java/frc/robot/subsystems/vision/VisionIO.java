package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  // tells AdvantageKit to log this class
  @AutoLog
  public static class VisionIOInputs {
    public boolean connected = false;
    public TargetObservation latestTargetObservation =
        new TargetObservation(new Rotation2d(), new Rotation2d());
    public PoseObservation[] poseObservations = new PoseObservation[0];
    public int[] tagIds = new int[0];
  }

  /** Represents the tx and ty angles to a simple target, not used for pose estimation. */
  /* record modifier creates an immutable class with implicit getters, reducing boilerplate */
  public static record TargetObservation(Rotation2d tx, Rotation2d ty) {}

  public static record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      PoseObservationType type) {}

  public static enum PoseObservationType {
    MEGATAG_1,
    MEGATAG_2,
  }

  // Method to update the inputs variable
  public default void updateInputs(VisionIOInputs inputs) {}
}
