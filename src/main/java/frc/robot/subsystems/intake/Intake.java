// Copyright (c) FIRST and other
// WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeConstants.PivotState;
import frc.robot.subsystems.intake.IntakeConstants.RollerState;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private RollerState rollerState;
  private PivotState pivotState;
  private final PIDController pivotController = new PIDController(0, 0, 0);

  public static final LoggedTunableNumber pivot_kP = new LoggedTunableNumber("Intake/pivot_kP");
  public static final LoggedTunableNumber pivot_kS = new LoggedTunableNumber("Intake/pivot_kS");
  public static final LoggedTunableNumber pivot_kG = new LoggedTunableNumber("Intake/pivot_kG");
  public static final LoggedTunableNumber pivot_kV = new LoggedTunableNumber("Intake/pivot_kV");
  public boolean isPivotStateZeroed;

  // final SysIdRoutine pivotSysIdRoutine; NO FEEFDFORWARD SO NO SYSID

  public Intake(IntakeIO io) {
    this.io = io;
    rollerState = RollerState.STOP;

    pivotState = PivotState.STOW;
  }

  @Override
  public void periodic() {
    switch (pivotState) {
      case STOW -> io.setPivotVoltage(
          pivotController.calculate(getPivotPosition(), PivotState.STOW.position));
      case DEPLOYING -> io.setPivotVoltage(
          pivotController.calculate(getPivotPosition(), PivotState.DEPLOYING.position));
      case DEPLOYED -> io.setPivotVoltage(
          pivotController.calculate(getPivotPosition(), PivotState.DEPLOYED.position));
      case OFF -> io.setPivotVoltage(
          pivotController.calculate(getPivotPosition(), PivotState.OFF.position));
      case UP -> io.setPivotVoltage(
          pivotController.calculate(getPivotPosition(), PivotState.UP.position));
    }

    io.updateInputs(inputs);
    Logger.processInputs("Roller", inputs);
    Logger.processInputs("Pivot", inputs);
    setDesiredPivotState(pivotState);
    setDesiredRollerState(rollerState);

    // Pivot
    if (Constants.tuningMode && (pivot_kP.hasChanged(hashCode()))
        || (pivot_kS.hasChanged(hashCode()))
        || (pivot_kG.hasChanged(hashCode()))
        || (pivot_kV.hasChanged(hashCode()))) {

      var pivotConfig = new SparkFlexConfig();
      pivotConfig.closedLoop.p(pivot_kP.get());
      io.updatePivotClosedLoopConfig(pivotConfig);
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
  public void establishPivotZero(boolean state) {
    isPivotStateZeroed = state;
  }

  public void setDesiredPivotState(PivotState pivotState) {
    this.pivotState = pivotState;
  }

  public void setPivotPosition(double targetPosition) {
    io.setPivotPosition(targetPosition);
  }

  public void setPivotVoltage(double targetVoltage) {
    io.setPivotVoltage(targetVoltage);
  }

  public void setPivotEncoderAngle(double targetAngleRadians) {
    io.setPivotEncoderAngle(targetAngleRadians);
  }

  // Roller Getter Methods
  public double getRollerVelocity() {
    return inputs.rollerRPM;
  }

  public double getRollerVoltage() {
    return inputs.rollerAppliedVoltage;
  }

  public double getRollerCurrent() {
    return inputs.rollerCurrent;
  }

  // Pivot Getter Methods
  public double getPivotPosition() {
    return inputs.pivotPositionRads;
  }

  public double getPivotVoltage() {
    return inputs.pivotAppliedVoltage;
  }

  public double getPivotCurrent() {
    return inputs.pivotCurrent;
  }

  public double getPivotVelocity() {
    return inputs.pivotVelocity;
  }

  // Roller Miscellaneous Methods
  public boolean pivotAtSetpoint(double targetPositionRadians) {
    final double tolerancePositionRadians = 0.05;
    return Math.abs(targetPositionRadians - inputs.pivotPositionRads) <= tolerancePositionRadians;
  }

  public void stopRollerMotor() {
    io.stopRollerMotor();
  }

  public void stopPivotMotor() {
    io.stopPivotMotor();
  }
}
