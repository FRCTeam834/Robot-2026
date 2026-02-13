// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
        Logger.recordOutput("Vision/PoseEstimate/CameraTimestampSeconds", inputs[i].timestampSeconds);
        Logger.recordOutput("Vision/PoseEstimate/CameraHasTarget", inputs[i].hasTarget);
      }
    }
  }
}