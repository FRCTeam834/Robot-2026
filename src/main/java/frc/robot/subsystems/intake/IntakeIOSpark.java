package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;

public class IntakeIOSpark implements IntakeIO {
  // Roller
  private SparkFlex rollerMotor;
  private RelativeEncoder rollerEncoder;
  private SparkFlexConfig rollerConfig = new SparkFlexConfig();

  // Pivot
  private SparkFlex pivotMotor;
  private SparkClosedLoopController pivotController;
  private RelativeEncoder pivotEncoder;

  private SparkFlexConfig pivotConfig = new SparkFlexConfig();

  public IntakeIOSpark() {
    // Roller
    rollerMotor = new SparkFlex(41, MotorType.kBrushless);
    rollerEncoder = rollerMotor.getEncoder();

    // Pivot
    pivotMotor = new SparkFlex(40, MotorType.kBrushless);
    pivotController = pivotMotor.getClosedLoopController();
    pivotEncoder = pivotMotor.getEncoder();

    pivotConfig
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .voltageCompensation(12.0)
        .smartCurrentLimit(40);

    // pivot config
    pivotConfig
        .encoder
        .positionConversionFactor((2 * Math.PI) / 36.666)
        .velocityConversionFactor((2 * Math.PI / 60) / 36.666);

    pivotConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0)
        .i(0)
        .d(0)
        .outputRange(-1, 1)
        .positionWrappingEnabled(false);

    rollerConfig.smartCurrentLimit(40).voltageCompensation(12).inverted(true);

    pivotMotor.configure(
        pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    pivotEncoder.setPosition(0);

    rollerMotor.configure(
        rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Roller
    inputs.rollerRPM = rollerEncoder.getVelocity();
    inputs.rollerAppliedVoltage = rollerMotor.getAppliedOutput() * rollerMotor.getBusVoltage();
    inputs.rollerCurrent = rollerMotor.getOutputCurrent();

    // Pivot
    inputs.pivotPositionRads = pivotEncoder.getPosition();
    inputs.pivotAppliedVoltage = pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage();
    inputs.pivotVelocity = pivotEncoder.getVelocity();
    inputs.pivotCurrent = pivotMotor.getOutputCurrent();
  }

  // Roller Methods
  @Override
  public void setRollerVoltage(double volts) {
    rollerMotor.setVoltage(volts);
  }

  // Pivot Methods
  @Override
  public void setPivotVoltage(double volts) {
    pivotMotor.setVoltage(volts);
  }

  @Override
  public void setPivotAngle(double angle) {
    angle = MathUtil.clamp(angle, 0, 2.6); // up is 0 and deployed is 2.5
    pivotController.setSetpoint(angle, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void setEncoderAngle(double angle) {
    pivotEncoder.setPosition(angle);
  }

  @Override
  public void updateClosedLoopConfig(SparkFlexConfig config) {
    pivotMotor.configureAsync(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void stopPivot() {
    pivotMotor.stopMotor();
  }

  @Override
  public void stopRollers() {
    rollerMotor.stopMotor();
  }
}
