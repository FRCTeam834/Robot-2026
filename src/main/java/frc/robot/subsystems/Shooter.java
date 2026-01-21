// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;


public class Shooter extends SubsystemBase {


  //akshaya added start

  //----------constants
  //Motor CAN IDs 
  private static final int SHOOTER_FLYWHEEL_ID = 9; //Kraken X60
  private static final int HOOD_PIVOT_ID = 10;      //Kraken X44
  
  //Flywheel PID (Velocity)
  private static final double FLYWHEEL_kP = 0.12;
  private static final double FLYWHEEL_kI = 0.0;
  private static final double FLYWHEEL_kD = 0.0;
  private static final double FLYWHEEL_kS = 0.0; //kV after SysId

  //Hood PID (position)
  private static final double HOOD_kP = 25.0;
  private static final double HOOD_kI = 0.0;
  private static final double HOOD_kD = 1.0;

  //Motor objects
  private final TalonFX shooterFlywheel;
  private final TalonFX hoodPivot;

  //Control Requests (using Slot0 PID constants for this request)
  private final VelocityVoltage flywheelVelocityRequest = new VelocityVoltage(0).withSlot(0);
  private final PositionVoltage hoodPositionRequest = new PositionVoltage(0).withSlot(0);
  
  //SysId
  private final SysIdRoutine flywheelSysId;

  /** Creates a new Shooter. */
  public Shooter() {
    //akshaya added start
    //Shooter Flywheel (x60)
    shooterFlywheel = new TalonFX(SHOOTER_FLYWHEEL_ID);

    //Hood Pivot (Kraken X44)
    hoodPivot = new TalonFX(HOOD_PIVOT_ID);

    TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
    flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    flywheelConfig.CurrentLimits.SupplyCurrentLimit = 40.0;

    flywheelConfig.Slot0.kP = FLYWHEEL_kP;
    flywheelConfig.Slot0.kI = FLYWHEEL_kI;
    flywheelConfig.Slot0.kD = FLYWHEEL_kD;
    flywheelConfig.Slot0.kV = FLYWHEEL_kS;

    shooterFlywheel.getConfigurator().apply(flywheelConfig);

    TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    hoodConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
  
    hoodConfig.Slot0.kP = HOOD_kP;
    hoodConfig.Slot0.kI = HOOD_kI;
    hoodConfig.Slot0.kD = HOOD_kD;
    //hoodConfig.Slot0.

    hoodPivot.getConfigurator().apply(hoodConfig);

    //SysId Routine (Flywheel)
    flywheelSysId = 
      new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
          //apply voltage
          volts -> {
            shooterFlywheel.setVoltage(volts.magnitude());
          },
          //log data
          log -> {
            log.motor("ShooterFlywheel")
              .voltage(
                Units.volts.of(
                  shooterFlywheel.getMotorVoltage().getValue()))
              .velocity(
                Units.RotationsPerSecond.of(
                  shooterFlywheel.getVelocity().getValue()))
          },
          this
        )
      );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //Shooter control methods

  //set flywheel target RPM using onboard PID
  public void shooterFlywheelRPM(double rpm){
    double rps = rpm / 60.0; //ctre uses rotations per second
    shooterFlywheel.setControl(flywheelVelocityRequest.withVelocity(rps));
  }

  //set hood position in motor rotations
  public void setHoodPosition(double rotations){
    hoodPivot.setControl(hoodPositionRequest.withPosition(rotations));
  }

  //stop all shooter motors
  public void stop(){
    shooterFlywheel.set(0);
    hoodPivot.set(0);
  }

  //SysId Commands (RETURN Command)

  public Command sysIdFlywheelQuasistaticForward(){
    return flywheelSysId.quasistatic(SysIdRoutine.Direction.kForward);
  }

  public Command sysIdFlywheelQuasistaticReverse(){
    return flywheelSysId.quasistatic(SysIdRoutine.Direction.kReverse);
  }

  public Command sysIdFlywheelDynamicForward(){
    return flywheelSysId.dynamic(SysIdRoutine.Direction.kForward);
  }

  public Command sysIdFlywheelDynamicReverse(){
    return flywheelSysId.dynamic(SysIdRoutine.Direction.kReverse);
  }

  
  //akshaya added end
}
// Kraken x60 for shooter flywheels
// kraken x44 for hood pivot
