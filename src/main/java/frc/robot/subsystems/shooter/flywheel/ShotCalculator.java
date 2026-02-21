// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShotCalculator {
  public static InterpolatingDoubleTreeMap shotSpeedTable = new InterpolatingDoubleTreeMap();

  static {
    // key: <horizontal distance meters>, value: <RPM>
    shotSpeedTable.put(0.0, 4000.0);
    shotSpeedTable.put(1.998, 4000.0);
    shotSpeedTable.put(2.235, 4000.0);
    shotSpeedTable.put(3.27, 4200.0);
    shotSpeedTable.put(3.976, 4600.0);
    shotSpeedTable.put(100.0, 4600.0);
  }

  public static double flywheelRPMForDistance(double meters) {
    return shotSpeedTable.get(meters);
  }
}
