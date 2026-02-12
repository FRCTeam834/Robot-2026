// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/** Add your docs here. */
public class ShotCalculator {
  // private static InterpolatingDoubleTreeMap shotAngleTable = new InterpolatingDoubleTreeMap();
  public static InterpolatingDoubleTreeMap shotSpeedTable = new InterpolatingDoubleTreeMap();

  static {
    /** key: <horizontal distance m>, value: <pivot angle rad> */
    // shotAngleTable.put(0.0, 0.75);
    // shotAngleTable.put(1.998, 0.743);
    // shotAngleTable.put(2.235, 0.7148);
    // shotAngleTable.put(2.576, 0.6854);
    // shotAngleTable.put(2.872, 0.6213);
    // shotAngleTable.put(3.27, 0.6);
    // shotAngleTable.put(3.508, 0.565);
    // shotAngleTable.put(3.976, 0.523);
    // shotAngleTable.put(100.0, 0.523);

    // key: <horizontal distance meters>, value: <RPM>
    shotSpeedTable.put(0.0, 4000.0);
    shotSpeedTable.put(1.998, 4000.0);
    shotSpeedTable.put(2.235, 4000.0);
    shotSpeedTable.put(3.27, 4200.0);
    shotSpeedTable.put(3.976, 4600.0);
    shotSpeedTable.put(100.0, 4600.0);
  }

  // public double angleForDist(double dist) {
  //   return shotAngleTable.get(dist);
  // }

  public static double flywheelRPMForDistance(double meters) {
    return shotSpeedTable.get(meters);
  }

  /*  public boolean atSetpoint(double distance) {
   double setpointRPM = shotSpeedTable.get(distance);
     double setpointAngle = shotAngleTable.get(distance);
     return Math.abs(setpointRPM - getCurrentTopRollerSpeed())
       && Math.abs(setpointRPM - getCurrentButtonRollerSpeed())
       && Math.abs(setpointAngle - getCurrentPivotAngle()) <= toleranceAngle;
  } */

}
