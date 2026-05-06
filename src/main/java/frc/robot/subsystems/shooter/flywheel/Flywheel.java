// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants.FlywheelState;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants;

public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  public static final LoggedTunableNumber flywheel_kP =
      new LoggedTunableNumber("Flywheel/flywheel_kP");
  public static final LoggedTunableNumber flywheel_kS =
      new LoggedTunableNumber("Flywheel/flywheel_kS");
  public static final LoggedTunableNumber flywheel_kV =
      new LoggedTunableNumber("Flywheel/flywheel_kV");

  final SysIdRoutine flywheelSysId;

  public Flywheel(FlywheelIO io) {
    this.io = io;

    // Flywheel SysId Routine
    flywheelSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("FlywheelSysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                (Voltage voltage) -> io.setFlywheelVoltage(voltage.in(Units.Volts)), null, this));
  }

  @Override
  public void periodic() {

    FlywheelState currentState = FlywheelState.STOP;
    switch (currentState) {
      case ACTIVE -> {

      },
      case MANUAL -> {
        
      },
      case IDLE -> {
        io.setFlywheelRPM(idleRPM);
      };
  }
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);

    if (Constants.tuningMode
        && (flywheel_kP.hasChanged(hashCode())
            || flywheel_kS.hasChanged(hashCode())
            || flywheel_kV.hasChanged(hashCode()))) {
      var flywheelConfig = new Slot0Configs();
      flywheelConfig.kP = flywheel_kP.get();
      flywheelConfig.kS = flywheel_kS.get();
      flywheelConfig.kV = flywheel_kV.get();
      io.updateFlywheelPID(flywheelConfig);
    }
  }

  // Flywheel SysId Commands
  public Command flywheelQuasistaticForward() {
    return flywheelSysId.quasistatic(SysIdRoutine.Direction.kForward);
  }

  public Command flywheezlQuasistaticReverse() {
    return flywheelSysId.quasistatic(SysIdRoutine.Direction.kReverse);
  }

  public Command sysIdFlywheelDynamicForward() {
    return flywheelSysId.dynamic(SysIdRoutine.Direction.kForward);
  }

  public Command sysIdFlywheelDynamicReverse() {
    return flywheelSysId.dynamic(SysIdRoutine.Direction.kReverse);
  }

  // Flywheel Getter Methods

  public double getLowerFlywheelVelocityRPM() {
    return inputs.lowerFlywheelVelocityRPM;
  }

  public double getLowerFlywheelCurrent(){
    return inputs.lowerFlywheelCurrent;
  }

  public double getLowerFlywheelDutyCycle() {
    return inputs.lowerFlywheelDutyCycle;
  }

    public double getUpperFlywheelVelocityRPM() {
    return inputs.upperFlywheelVelocityRPM;
  }

  public double getUpperFlywheelCurrent(){
    return inputs.upperFlywheelCurrent;
  }

  public double getUpperFlywheelDutyCycle() {
    return inputs.upperFlywheelDutyCycle;
  }

  // Flywheel Setter Methods
  public void setFlywheelVoltage(double targetVolts) {
    io.setFlywheelVoltage(targetVolts);
  }

  public void setFlywheelRPM(double targetRPM) {
    io.setFlywheelVelocity(targetRPM);
  }

  public void setFlywheelVelocityForDistance(double distanceMeters) {
    double targetRPMSetpoint = ShotCalculator.flywheelRPMForDistance(distanceMeters);
    io.setFlywheelVelocity(targetRPMSetpoint); 
  } 
  
  // Miscellaneous Methods
  public boolean flywheelRPMAtSetpoint(double targetRPM) {
    return Math.abs(targetRPM - inputs.upperFlywheelVelocityRPM) <= FlywheelConstants.toleranceRPM;
  }

  public void stopMotor() {
    io.stopMotor();
  }

  
}
