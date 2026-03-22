// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Flywheel extends SubsystemBase {
  /** Creates a new Flywheel. */
  private final TalonFX mainFlywheel;
  private final TalonFX upperFlywheel;

  private final VoltageOut sysIdRequest = new VoltageOut(0);
  private final DutyCycleOut joystickRequest = new DutyCycleOut(0);
  private final SysIdRoutine m_sysIdRoutine;

  public Flywheel() {
    setName("Flywheel");

    mainFlywheel = new TalonFX(30);
    upperFlywheel = new TalonFX(32);

    m_sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,         // Use default ramp rate (1 V/s)
                Volts.of(4), // Reduce dynamic voltage to 4 to prevent brownout
                null,          // Use default timeout (10 s)
                                       // Log state with Phoenix SignalLogger class
                state -> SignalLogger.writeString("state", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                volts -> mainFlywheel.setControl(sysIdRequest.withOutput(volts)),
                null,
                this
            )
        );

    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    mainFlywheel.getConfigurator().apply(cfg);
    upperFlywheel.getConfigurator().apply(cfg);

    BaseStatusSignal.setUpdateFrequencyForAll(250, 
    mainFlywheel.getPosition(),
    mainFlywheel.getVelocity(),
    mainFlywheel.getMotorVoltage());
    mainFlywheel.optimizeBusUtilization();

    SignalLogger.start();

    upperFlywheel.setControl(new Follower(mainFlywheel.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("upper duty cycle", upperFlywheel.getDutyCycle().getValueAsDouble());
  }

  public Command joystickDriveCommand(DoubleSupplier output) {
        return run(() -> mainFlywheel.setControl(joystickRequest.withOutput(output.getAsDouble())));
    }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }
}
