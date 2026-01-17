// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;


public class Shooter extends SubsystemBase {


  //akshaya added start

  //----------constants
  //Motor CAN IDs 
  private static final int SHOOTER_FLYWHEEL_ID = 9; //Kraken X60
  private static final int HOOD_PIVOT_ID = 10;      //Kraken X44

  //Motor objects
  private final TalonFX shooterFlywheel;
  private final TalonFX hoodPivot;
  //akshaya added end

  /** Creates a new Shooter. */
  public Shooter() {
    //akshaya added start
    //Shooter Flywheel (x60)
    shooterFlywheel = new TalonFX(SHOOTER_FLYWHEEL_ID);

    TalonFXConfiguration flywheelConfig = new TalonFXConfiguration();
    flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    flywheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    flywheelConfig.CurrentLimits.SupplyCurrentLimit = 40.0;

    shooterFlywheel.getConfigurator().apply(flywheelConfig);

    //Hood Pivot (Kraken X44)
    hoodPivot = new TalonFX(HOOD_PIVOT_ID);

    TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    hoodConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
  
    hoodPivot.getConfigurator().apply(hoodConfig);
    //akshaya added code end
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  //akshaya added start
  public void shooterFlywheelPower(double power){
    shooterFlywheel.set(power);
  }

  public void setHoodPower(double power){
    hoodPivot.set(power);
  }

  public void stop(){
    shooterFlywheel.set(0);
    hoodPivot.set(0);
  }
  //PID to be added

  //Flywheel PID (Velocity)
  private static final double FLYWHEEL_kP = 0.12;
  private static final double FLYWHEEL_kI = 0.0;
  private static final double FLYWHEEL_kD = 0.0;
  private static final double FLYWHEEL_kF = 0.0;

  //Hood PID (position)
  private static final double HOOD_kP = 25.0;
  private static final double HOOD_kI = 0.0;
  private static final double HOOD_kD = 1.0;

  //Control Requests
  private final VelocityVoltage flywheelVelocityRequest = new VelocityVoltage(0);
  private final PositionVoltage hoodPositionRequest = new PositionVoltage(0);

  //akshaya added end
}
// Kraken x60 for shooter flywheels
// kraken x44 for hood pivot
