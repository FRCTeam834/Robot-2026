// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  // List of hardware
  private final VisionIO[] io;
  // Data storage list
  private final VisionIOInputsAutoLogged[] inputs;

  public Vision(VisionIO... io) {
    this.io = io;
    // storage size
    this.inputs = new VisionIOInputsAutoLogged[io.length];

    // Create each data folder
    for (int i = 0; i < io.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }
  }

  @Override
  public void periodic() {
    // Loop through hardware devices (Cameras)
    for (int i = 0; i < io.length; i++) {
      // Get new data
      io[i].updateInputs(inputs[i]);
      // Save to logger
      Logger.processInputs("Vision/Camera" + i, inputs[i]);
      // If target is found, record to AdvantageScope
      if (inputs[i].poseEstimates.length > 0) {
        // Record array to AdvantageScope
        Logger.recordOutput("Vision/PoseEstimates/Camera" + i, inputs[i].poseEstimates);
      }
    }
  }
}
