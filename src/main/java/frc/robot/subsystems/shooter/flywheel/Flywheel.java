// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants.FlywheelState;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  public static final LoggedTunableNumber flywheel_kP =
      new LoggedTunableNumber("Flywheel/flywheel_kP");
  public static final LoggedTunableNumber flywheel_kS =
      new LoggedTunableNumber("Flywheel/flywheel_kS");
  public static final LoggedTunableNumber flywheel_kV =
      new LoggedTunableNumber("Flywheel/flywheel_kV");

  @AutoLogOutput(key = "SubsystemStates/FlywheelState")
  private FlywheelState flywheelState = FlywheelState.STOPPED;

  public Flywheel(FlywheelIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
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
      io.updateClosedLoopConfig(flywheelConfig);
    }
  }

  public double getFlywheelRPM() {
    return inputs.flywheelVelocityRPM;
  }

  public void setFlywheelRPM(double targetRPM) {
    io.setFlywheelVelocity(targetRPM);
  }

  public void setFlywheelVelocityForDistance(double distanceMeters) {
    double setpointVelocity = ShotCalculator.flywheelRPMForDistance(distanceMeters);
    io.setFlywheelVelocity(setpointVelocity);
    flywheelState = FlywheelState.ACTIVE;
  }

  // Miscellaneous Methods
  public boolean isAtSetpointRPM(double targetRPM) {
    return Math.abs(targetRPM - inputs.flywheelVelocityRPM) <= FlywheelConstants.toleranceRPM;
  }

  public void setIdleSpeed() {
    io.setFlywheelVelocity(FlywheelConstants.idleRPM);
    flywheelState = FlywheelState.IDLE;
  }

  public void stopMotor() {
    io.stopMotor();
    flywheelState = FlywheelState.STOPPED;
  }

  public FlywheelState getState() {
    return flywheelState;
  }
}
