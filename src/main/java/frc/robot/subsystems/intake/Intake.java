// Copyright (c) FIRST and other
// WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.spark.config.ClosedLoopConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeConstants.PivotState;
import frc.robot.subsystems.intake.IntakeConstants.RollerState;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  @AutoLogOutput(key = "SubsystemStates/rollerState") private RollerState rollerState = RollerState.STOP;
  @AutoLogOutput(key = "SubsystemStates/pivotState") private PivotState pivotState = PivotState.STOW;

  public static final LoggedTunableNumber pivot_kP = new LoggedTunableNumber("Intake/pivot_kP");
  public static final LoggedTunableNumber pivot_kS = new LoggedTunableNumber("Intake/pivot_kS");
  public static final LoggedTunableNumber pivot_kG = new LoggedTunableNumber("Intake/pivot_kG");
  public static final LoggedTunableNumber pivot_kV = new LoggedTunableNumber("Intake/pivot_kV");

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    // Pivot
    if (Constants.tuningMode
        && (pivot_kP.hasChanged(hashCode())
            || pivot_kS.hasChanged(hashCode())
            || pivot_kV.hasChanged(hashCode())
            || pivot_kG.hasChanged(hashCode()))) {

      var pivotConfig = new ClosedLoopConfig();
      pivotConfig.p(pivot_kP.get());
      pivotConfig.feedForward.kS(pivot_kS.get()).kV(pivot_kV.get()).kG(pivot_kG.get());

      io.updateClosedLoopConfig(pivotConfig);
    }
  }

  // Roller Setter Methods
  public void setRollerState(RollerState rollerState) {
    this.rollerState = rollerState;
    setRollerVoltage(rollerState.voltage);
  }

  public void setRollerVoltage(double targetVolts) {
    io.setRollerVoltage(targetVolts);
  }

  // Pivot Setter Methods
  public void setPivotState(PivotState pivotState) {
    this.pivotState = pivotState;
    setPivotAngle(pivotState.position);
  }

  public void setPivotVoltage(double targetVolts) {
    io.setPivotVoltage(targetVolts);
  }

  public void setPivotAngle(double angle) {
    io.setPivotAngle(angle);
  }

  // Roller Getter Methods
  public double getCurrentRollerVelocity() {
    return inputs.rollerRPM;
  }

  public double getCurrentRollerVoltage() {
    return inputs.rollerAppliedVoltage;
  }

  // Pivot Getter Methods
  public double getCurrentPivotPosition() {
    return inputs.pivotPositionRads;
  }

  public double getCurrentPivotVoltage() {
    return inputs.pivotAppliedVoltage;
  }

  public double getPivotVelocity() {
    return inputs.pivotRPM;
  }

  public void stopMotors() {
    io.stopMotors();
  }
}
