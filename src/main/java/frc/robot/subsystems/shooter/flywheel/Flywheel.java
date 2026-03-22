// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.flywheel.FlywheelConstants.FlywheelState;
import frc.robot.util.LoggedTunableNumber;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private DoubleSupplier hubDistance;

  public static final LoggedTunableNumber flywheel_kP =
      new LoggedTunableNumber("Flywheel/flywheel_kP");
  public static final LoggedTunableNumber flywheel_kS =
      new LoggedTunableNumber("Flywheel/flywheel_kS");
  public static final LoggedTunableNumber flywheel_kV =
      new LoggedTunableNumber("Flywheel/flywheel_kV");

  @AutoLogOutput(key = "Flywheel/SetpointRPM")
  private double flywheelSetpointRPM;

  @AutoLogOutput(key = "SubsystemStates/FlywheelState")
  private FlywheelState flywheelState;
  
  private final SysIdRoutine m_sysIdRoutine;

  static {
    flywheel_kP.initDefault(5);
    flywheel_kS.initDefault(3.9);
    flywheel_kV.initDefault(0.045);
  }

  public Flywheel(FlywheelIO io, DoubleSupplier hubDistance) {
    this.io = io;
    this.hubDistance = hubDistance;

    m_sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,         // Use default ramp rate (1 V/s)
                null,     // Use default dynamic voltage of 7
                null,          // Use default timeout (10 s)
                                       // Log state with Phoenix SignalLogger class
                state -> Logger.recordOutput("Flywheel/SysIdState", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                volts -> runVolts(volts.in(Volts)),
                null,
                this
            )
        );

    flywheelState = FlywheelState.STOPPED;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);

    if (Constants.TUNING_MODE
        && (flywheel_kP.hasChanged(hashCode())
            || flywheel_kS.hasChanged(hashCode())
            || flywheel_kV.hasChanged(hashCode()))) {
      var flywheelConfig = new Slot0Configs();
      flywheelConfig.kP = flywheel_kP.get();
      flywheelConfig.kS = flywheel_kS.get();
      flywheelConfig.kV = flywheel_kV.get();
      io.updateClosedLoopConfig(flywheelConfig);
    }

    switch (flywheelState) {
      case ACTIVE -> {
        flywheelSetpointRPM = ShotCalculator.flywheelRPMForDistance(hubDistance.getAsDouble());
        io.setFlywheelVelocity(flywheelSetpointRPM);
      }
      case IDLE -> io.setFlywheelVelocity(FlywheelConstants.idleRPM);
      case MANUAL_RPM -> io.setFlywheelVelocity(flywheelSetpointRPM);
      case STOPPED -> io.stopMotor();
      case SYSID -> {}
    }
  }

  public double getFlywheelRPM() {
    return inputs.ONE_flywheelVelocityRPM;
  }

  public void setDesiredState(FlywheelState state) {
    flywheelState = state;
  }

  public void setVelocitySetpoint(double rpm) {
    if (flywheelState == FlywheelState.ACTIVE) return;
    rpm = MathUtil.clamp(rpm, 0, 3000);
    flywheelSetpointRPM = rpm;
  }

  public void runVolts(double volts) {
    io.setFlywheelVoltage(volts);
  }

  public double getVelocitySetpoint() {
    return flywheelSetpointRPM;
  }

  // Miscellaneous Methods
  public boolean isAtSetpointRPM() {
    return MathUtil.isNear(flywheelSetpointRPM, getFlywheelRPM(), FlywheelConstants.toleranceRPM);
  }

  public FlywheelState getState() {
    return flywheelState;
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return runOnce(() -> setDesiredState(FlywheelState.SYSID)).andThen(m_sysIdRoutine.quasistatic(direction));
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return runOnce(() -> setDesiredState(FlywheelState.SYSID)).andThen(m_sysIdRoutine.dynamic(direction));
  }
}
