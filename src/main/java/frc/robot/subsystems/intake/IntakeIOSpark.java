package frc.robot.subsystems.intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

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


    rollerConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder).pid(0, 0, 0);

    pivotConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder).pid(0, 0, 0);

    rollerConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12.0);

    pivotConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12.0);

    rollerEncoderConfig
        .positionConversionFactor(2 * Math.PI)
        .velocityConversionFactor(2 * Math.PI / 60);

    pivotEncoderConfig
        .zeroOffset(0)
        .positionConversionFactor(2 * Math.PI)
        .velocityConversionFactor(2 * Math.PI / 60);

    rollerMotor.configure(
        rollerConfig,
        com.revrobotics.ResetMode.kNoResetSafeParameters,
        com.revrobotics.PersistMode.kNoPersistParameters);

    pivotMotor.configure(
        pivotConfig,
        com.revrobotics.ResetMode.kNoResetSafeParameters,
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
    pivotSetpoint =
        pivotProfile.calculate(
            0.020, pivotSetpoint, new TrapezoidProfile.State(targetPositionRads, 0));
    double ffVolts = pivotFeedforward.calculate(pivotSetpoint.position, pivotSetpoint.velocity);
    pivotController.setSetpoint(
        pivotSetpoint.position, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0, ffVolts);
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
  public void stopRollerMotor(){
    rollerMotor.stopMotor();
  }

  @Override
  public void stopPivotMotor(){
    pivotMotor.stopMotor();
  }

    // @Override
  // public void updatePivotFeedforward(double kS, double kG, double kV) {
  //  this.pivotFeedforward = new ArmFeedforward(kS, kG, kV);
  // }

}
