package frc.robot.subsystems.intake;

import com.revrobotics.RelativeEncoder;
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
  private SparkFlexConfig rollerConfig;

  // Pivot
  private SparkFlex pivotMotor;
  private SparkClosedLoopController pivotController;
  private RelativeEncoder pivotEncoder;
  private SparkFlexConfig pivotConfig;

  public IntakeIOSpark() {
    // Roller
    rollerMotor = new SparkFlex(11, MotorType.kBrushless);
    rollerEncoder = rollerMotor.getEncoder();
    rollerConfig = new SparkFlexConfig();

    // Pivot
    pivotMotor = new SparkFlex(12, MotorType.kBrushless);
    pivotEncoder = pivotMotor.getEncoder();
    pivotConfig = new SparkFlexConfig();
    pivotController = pivotMotor.getClosedLoopController();
    // pivotFeedforward = new ArmFeedforward(0, 0, 0);

    rollerConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(40)
        .voltageCompensation(12)
        .inverted(true); // NOT SURE IF TRUE OR FALSE. MUST TEST EMPIRICALLY

    pivotConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40)
        .voltageCompensation(12)
        .inverted(true); // MUST BE CHANGED LATER

    pivotConfig
        .encoder
        .positionConversionFactor(2.0 * Math.PI) // 2pi/gear ratio; MUST BE CHANGED LATER
        .velocityConversionFactor(2.0 * Math.PI / 60) // (2pi/60)/gear ratio; MUST BE CHANGED LATER
        .inverted(true); // MUST BE CHANGED LATER

    pivotConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0)
        .i(0)
        .d(0)
        .outputRange(-1, 1)
        .positionWrappingEnabled(false);

    rollerMotor.configure(
        rollerConfig,
        com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kNoPersistParameters); // Change when done tuning

    pivotMotor.configure(
        pivotConfig,
        com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kNoPersistParameters);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Roller
    inputs.rollerRPM = rollerEncoder.getVelocity();
    inputs.rollerAppliedVoltage = rollerMotor.getBusVoltage();
    inputs.rollerCurrent = rollerMotor.getOutputCurrent();

    // Pivot
    inputs.pivotPositionRads = pivotEncoder.getPosition();
    inputs.pivotVelocity = pivotEncoder.getVelocity();
    inputs.pivotAppliedVoltage = pivotMotor.getBusVoltage();
    inputs.pivotCurrent = pivotMotor.getOutputCurrent();
  }

  // Roller Methods
  @Override
  public void setRollerVoltage(double targetVolts) {
    rollerMotor.setVoltage(MathUtil.clamp(targetVolts, -12.0, 12.0));
  }

  // Pivot Methods
  @Override
  public void setPivotVoltage(double volts) {
    pivotMotor.setVoltage(MathUtil.clamp(volts, -12.0, 12.0));
  }

  @Override
  public void setPivotPosition(double targetPositionRads) {
    pivotController.setSetpoint(targetPositionRads, ControlType.kPosition);
  }

  @Override
  public void updatePivotClosedLoopConfig(SparkFlexConfig config) {
    this.pivotConfig = config;
    pivotMotor.configure(
        pivotConfig,
        com.revrobotics.ResetMode.kNoResetSafeParameters,
        com.revrobotics.PersistMode.kNoPersistParameters);
  }

  @Override
  public void stopRollerMotor() {
    rollerMotor.stopMotor();
  }

  @Override
  public void stopPivotMotor() {
    pivotMotor.stopMotor();
  }

  // @Override
  // public void updatePivotFeedforward(double kS, double kG, double kV) {
  //  this.pivotFeedforward = new ArmFeedforward(kS, kG, kV);
  // }

}
