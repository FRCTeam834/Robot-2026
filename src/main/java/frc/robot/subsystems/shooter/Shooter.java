// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private final ShooterIO io; // The IO Implementation; transfers data to/from hardware
  private final ShooterIOInputsAutoLogged inputs =
      new ShooterIOInputsAutoLogged(); // The IO Layer; Transfers Data; Fills Class w/ Numbers every
  // 20ms

  // Two Separate SysId Routines
  private final SysIdRoutine flywheelSysId;
  private final SysIdRoutine hoodSysId;

  //Interpolating Tables
  private static InterpolatingDoubleTreeMap shotAngleTable = new InterpolatingDoubleTreeMap;
  private static InterpolatingDoubleTreeMap shotSpeedTable = new InterpolatingDoubleTreeMap;
  private static double hoodPivotAngle = Units.degreesToRadians(0.95);  // does this have to do with the interpolating table gabe?

  //initializing values for shot table -- THESE #s ARE IRRELEVANT AND NOT TESTED
  static {
    /** key: <horizontal distance m>, value: <pivot angle rad> */
    shotAngleTable.put(0.0, 0.75);
    shotAngleTable.put(1.998, 0.743);
    shotAngleTable.put(2.235, 0.7148);
    shotAngleTable.put(2.576, 0.6854); 
    shotAngleTable.put(2.872, 0.6213); 
    shotAngleTable.put(3.27, 0.6);
    shotAngleTable.put(3.508, 0.565);
    shotAngleTable.put(3.976, 0.523);
    shotAngleTable.put(100.0, 0.523);
    
    /** key: <horizontal distance m>, value: <rpm> */
    
    shotSpeedTable.put(0.0, 4000.0);
    shotSpeedTable.put(1.998, 4000.0);
    shotSpeedTable.put(2.235, 4000.0);
    shotSpeedTable.put(3.27, 4200.0);
    shotSpeedTable.put(3.976, 4600.0);
    shotSpeedTable.put(100.0, 4600.0);
    
  }
  
  // Shooter Constructor
  public Shooter(ShooterIO io) {
    this.io = io;

    // Flywheel SysId Routine
    flywheelSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null, // Default config
                (state) -> Logger.recordOutput("FlywheelSysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setFlywheelVoltage(voltage.in(Units.Volts)),
                null, // No log consumer, data recorded by AdvantageKit
                this));

    // Hood SysId Routine
    hoodSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null, // Default config
                (state) -> Logger.recordOutput("HoodSysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setHoodVoltage(voltage.in(Units.Volts)),
                null, // No log consumer, data recorded by AdvantageKit
                this));
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  // Flywheel SysId Commands
  public Command flywheelQuasistaticForward() {
    return flywheelSysId.quasistatic(SysIdRoutine.Direction.kForward);
  }

  public Command flywheelQuasistaticReverse() {
    return flywheelSysId.quasistatic(SysIdRoutine.Direction.kReverse);
  }

  public Command sysIdFlywheelDynamicForward() {
    return flywheelSysId.dynamic(SysIdRoutine.Direction.kForward);
  }

  public Command sysIdFlywheelDynamicReverse() {
    return flywheelSysId.dynamic(SysIdRoutine.Direction.kReverse);
  }

  // Hood SysId Commands
  public Command hoodQuasistaticForward() {
    return hoodSysId.quasistatic(SysIdRoutine.Direction.kForward);
  }

  public Command hoodQuasistaticReverse() {
    return hoodSysId.quasistatic(SysIdRoutine.Direction.kReverse);
  }

  public Command hoodDynamicForward() {
    return hoodSysId.dynamic(SysIdRoutine.Direction.kForward);
  }

  public Command hoodDynamicReverse() {
    return hoodSysId.dynamic(SysIdRoutine.Direction.kReverse);
  }


  public boolean atSetpoint (double distance) {
    double setpointRPM = shotSpeedTable.get(distance);
    double setpointAngle = shotAngleTable.get(distance);
    return
        Math.abs(setpointRPM - getCurrentTopRollerSpeed()) &&
        Math.abs(setpointRPM - getCurrentButtonRollerSpeed()) &&
        Math.abs(setpointAngle - getCurrentPivotAngle()) <= toleranceAngle;
  }//will add all of these methods next meeting - Daniel
}



// need to do main logic next meeting
//interpolation - check 2024 robot
//class InterpolationDoubleTreeMap
