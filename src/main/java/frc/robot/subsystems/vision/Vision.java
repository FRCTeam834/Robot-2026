// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {

  // holds all camera IO interfaces
  private final VisionIO[] io;

  // holds logged inputs for each camera
  private final VisionIOInputsAutoLogged[] inputs;

  // any number of IO inputs
  public Vision(VisionIO... io) {

    this.io = io;

    // inputs array with the same size
    this.inputs = new VisionIOInputsAutoLogged[io.length];

    // create empty input object for each camera
    for (int i = 0; i < io.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }
  }

  @Override
  public void periodic() {

    // Loop through every camera
    for (int i = 0; i < io.length; i++) {
      // Refresh data from hardware
      io[i].updateInputs(inputs[i]);

      // Send all raw data to the logger
      Logger.processInputs("Vision/Camera" + i, inputs[i]);

      // Check if camera actually sees target
      if (inputs[i].hasTarget && inputs[i].tagCount > 0) {
        Logger.recordOutput("Vision/PoseEstimate/CameraPose" + i, inputs[i].pose);
        Logger.recordOutput("Vision/PoseEstimate/CameraTagCount" + i, inputs[i].tagCount);
        Logger.recordOutput(
            "Vision/PoseEstimate/CameraTimestampSeconds", inputs[i].timestampSeconds);
        Logger.recordOutput("Vision/PoseEstimate/CameraHasTarget", inputs[i].hasTarget);
      }
    }
  }

  // Returns true if any camera reports hasTarget == true and tagCount > 0;
  // otherwise false
  public boolean hasAnyTarget() {
    for (VisionIOInputsAutoLogged in : inputs) {
      if (in.hasTarget && in.tagCount > 0) {
        return true;
      }
    }
    return false;
  }

  // Scans cached inputs and returns the Pose2d from the camera with the newest timestamp that also
  // reports a valid target (hasTarget && tagCount > 0)
  // Return type: Optional<Pose2d>. Returns Optional.empty() if no valid pose is available.
  // Optional: May or may not have a null value
  public Optional<Pose2d> getBestPoseEstimate() { 
    int bestIndex = -1; // Start with an invalid index because we haven't found a valid pose yet
    double bestTimestamp =
        Double.NEGATIVE_INFINITY; // Start with the lowest possible timestamp so any valid pose will
    // be newer
    for (int i = 0; i < inputs.length; i++) {
      // Check if this camera has a valid target and is newer than the best valid pose we've seen so
      // far
      if (inputs[i].hasTarget
          && inputs[i].tagCount > 0
          && inputs[i].timestampSeconds > bestTimestamp) {
        bestTimestamp = inputs[i].timestampSeconds;
        bestIndex = i;
      }
    }
    if (bestIndex >= 0) {
      return Optional.of(inputs[bestIndex].pose);
    }
    return Optional.empty();
  }

  // Uses Timer.getFPGATimestamp() to get age of each camera pose.
  // Returns the newest pose whose age <= maxAgeSeconds (and that has a valid target)
  // Optional: May or may not have a null value. Returns Optional.empty() if no valid pose is
  // available.
  public Optional<Pose2d> getPoseIfFresh(double maxAgeSeconds) {
    double current = Timer.getFPGATimestamp(); // Get current time to calculate age of each pose
    Optional<Pose2d> best =
        Optional.empty(); // Start with empty Optional because we haven't found a valid pose yet
    double bestTimestamp =
        Double.NEGATIVE_INFINITY; // Start with the lowest possible timestamp so any valid pose will
    // be newer
    for (var in : inputs) {
      if (in.hasTarget && in.tagCount > 0) { // Check if this camera has a valid target
        double age = current - in.timestampSeconds; // Calculate age of this pose
        // Check if this pose is newer than the best valid pose we've seen so far
        if (age <= maxAgeSeconds && in.timestampSeconds > bestTimestamp) {
          bestTimestamp = in.timestampSeconds;
          best = Optional.of(in.pose); // Update best pose
        }
      }
    }
    return best;
  }

  // # of cameras
  public int getCameraCount() { 
    return io.length;
  }

  // Returns a copy of the VisionIOInputs for the camera at the given index
  public Optional<VisionIO.VisionIOInputs> getCameraInputsCopy(int index) {
    if (index < 0 || index >= inputs.length) {
      return Optional.empty(); // Return empty Optional if index is out of bounds
    }

    // Create a shallow copy of the logged inputs
    var src = inputs[index];
    var dst = new VisionIO.VisionIOInputs();
    dst.hasTarget = src.hasTarget;
    dst.tagCount = src.tagCount;
    dst.pose = src.pose;
    dst.timestampSeconds = src.timestampSeconds;
    return Optional.of(dst);
  }
}
