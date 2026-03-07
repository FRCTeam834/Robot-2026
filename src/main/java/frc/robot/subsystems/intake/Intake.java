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

  public static final LoggedTunableNumber pivot_kP = new LoggedTunableNumber("Intake/pivot_kP");
  public static final LoggedTunableNumber pivot_kS = new LoggedTunableNumber("Intake/pivot_kS");
  public static final LoggedTunableNumber pivot_kCos = new LoggedTunableNumber("Intake/pivot_kCos");
  public static final LoggedTunableNumber pivot_kV = new LoggedTunableNumber("Intake/pivot_kV");

  @AutoLogOutput(key = "SubsystemStates/rollerState")
  private RollerState rollerState;

  @AutoLogOutput(key = "SubsystemStates/pivotState")
  private PivotState pivotState;

  private boolean isPivotZeroed;

  static {
    pivot_kP.initDefault(0);
    pivot_kS.initDefault(0);
    pivot_kCos.initDefault(0);
    pivot_kV.initDefault(0);
  }

  public Intake(IntakeIO io) {
    this.io = io;
    rollerState = RollerState.STOP;
    pivotState = PivotState.STOW;
    isPivotZeroed = true;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    // Pivot Tuning
    if (Constants.TUNING_MODE
        && (pivot_kP.hasChanged(hashCode())
            || pivot_kS.hasChanged(hashCode())
            || pivot_kV.hasChanged(hashCode())
            || pivot_kCos.hasChanged(hashCode()))) {

      var pivotConfig = new ClosedLoopConfig();
      pivotConfig.p(pivot_kP.get());
      pivotConfig.feedForward.kS(pivot_kS.get()).kV(pivot_kV.get()).kG(pivot_kCos.get());

      io.updateClosedLoopConfig(pivotConfig);
    }

    // handle the transition from deploying to deployed
    if (pivotState == PivotState.DEPLOYING
        && pivotAtSetpoint(PivotState.DEPLOYED, IntakeConstants.pivotTolerance)) {
      stopPivot();
      pivotState = PivotState.DEPLOYED;
    }
  }

  // Roller Setter Methods
  public void setDesiredRollerState(RollerState rollerState) {
    this.rollerState = rollerState;
    setRollerVoltage(rollerState.voltage);
  }

  public void setRollerVoltage(double targetVolts) {
    io.setRollerVoltage(targetVolts);
  }

  // Pivot Setter Methods
  public void setDesiredPivotState(PivotState pivotState) {
    switch (pivotState) {
      case DEPLOYING -> io.setPivotAngle(PivotState.DEPLOYING.position);
      case UP -> io.setPivotAngle(PivotState.UP.position);
      case STOW -> io.setPivotAngle(PivotState.STOW.position);
      case DEPLOYED -> {}
    }
    this.pivotState = pivotState;
  }

  public void setPivotVoltage(double targetVolts) {
    io.setPivotVoltage(targetVolts);
  }

  public boolean isPivotZeroed() {
    return isPivotZeroed;
  }

  public void establishPivotZero(boolean state) {
    isPivotZeroed = state;
  }

  public void setEncoderAngle(double angle) {
    io.setEncoderAngle(angle);
  }

  // Roller Getter Methods
  public double getCurrentRollerVelocity() {
    return inputs.rollerRPM;
  }

  public double getCurrentRollerVoltage() {
    return inputs.rollerAppliedVoltage;
  }

  // Pivot Getter Methods
  public double getCurrentPivotAngle() {
    return inputs.pivotPositionRads;
  }

  public double getCurrentPivotVoltage() {
    return inputs.pivotAppliedVoltage;
  }

  public double getPivotVelocity() {
    return inputs.pivotRPM;
  }

  public void setPivotAngle(double angle) {
    io.setPivotAngle(angle);
  }

  public double getPivotCurrent() {
    return inputs.pivotCurrent;
  }

  public PivotState getPivotState() {
    return pivotState;
  }

  public RollerState getRollersState() {
    return rollerState;
  }

  public boolean pivotAtSetpoint(PivotState state, double toleranceRads) {
    return Math.abs(getCurrentPivotAngle() - state.position) <= toleranceRads;
  }

  public void stopPivot() {
    io.stopPivot();
  }

  public void stopRollers() {
    io.stopRollers();
  }
}
