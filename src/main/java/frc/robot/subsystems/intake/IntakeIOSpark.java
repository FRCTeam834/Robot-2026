package frc.robot.subsystems.intake;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.ArmFeedforward;

public class IntakeIOSpark implements IntakeIO {
  // Roller
  private SparkFlex rollerMotor;
  private SparkAbsoluteEncoder rollerEncoder;
  private SparkFlexConfig rollerConfig;
  private AbsoluteEncoderConfig rollerEncoderConfig;
  private SimpleMotorFeedforward rollerFeedforward;
  private SparkClosedLoopController rollerController;

  // Pivot
  private SparkMax pivotMotor;
  private SparkAbsoluteEncoder pivotEncoder;
  private SparkMaxConfig pivotConfig;
  private AbsoluteEncoderConfig pivotEncoderConfig;
  private double pivotVolts;
  private ArmFeedforward pivotFeedforward;
  private TrapezoidProfile pivotProfile;
  private TrapezoidProfile.State pivotSetpoint;
  private SparkClosedLoopController pivotController;

  public IntakeIOSpark() {
    // Roller
    rollerMotor = new SparkFlex(11, MotorType.kBrushless);
    rollerEncoder = rollerMotor.getAbsoluteEncoder();
    rollerConfig = new SparkFlexConfig();
    rollerFeedforward = new SimpleMotorFeedforward(0, 0);
    rollerController = rollerMotor.getClosedLoopController();

    // Pivot
    pivotMotor = new SparkMax(12, MotorType.kBrushless);
    pivotEncoder = pivotMotor.getAbsoluteEncoder();
    pivotConfig = new SparkMaxConfig();
    pivotEncoderConfig = new AbsoluteEncoderConfig();
    pivotFeedforward = new ArmFeedforward(0, 0, 0);
    pivotProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(5, 10));
    pivotSetpoint = new TrapezoidProfile.State();
    pivotController = pivotMotor.getClosedLoopController();
 
    rollerConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
    .pid(0, 0, 0);

    pivotConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
    .pid(0, 0, 0);

    rollerConfig
    .idleMode(IdleMode.kCoast)
    .smartCurrentLimit(40)
    .voltageCompensation(12.0);

    pivotConfig
    .idleMode(IdleMode.kBrake)
    .smartCurrentLimit(40)
    .voltageCompensation(12.0);

    rollerEncoderConfig
    .positionConversionFactor(2 * Math.PI)
    .velocityConversionFactor(2 * Math.PI / 60);

    pivotEncoderConfig
    .zeroOffset(0) 
    .positionConversionFactor(2 * Math.PI)
    .velocityConversionFactor(2 * Math.PI / 60);

    rollerConfig.apply(rollerEncoderConfig);

    pivotConfig.apply(pivotEncoderConfig);

    rollerMotor.configure(rollerConfig, 
      com.revrobotics.ResetMode.kNoResetSafeParameters,
      com.revrobotics.PersistMode.kNoPersistParameters);

    pivotMotor.configure(pivotConfig, 
      com.revrobotics.ResetMode.kNoResetSafeParameters,
      com.revrobotics.PersistMode.kNoPersistParameters);
  }


  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Roller
    inputs.rollerConnected = true;
    inputs.rollerRPM = rollerEncoder.getVelocity();
    inputs.rollerAppliedVoltage = rollerMotor.getBusVoltage();

    // Pivot
    inputs.pivotConnected = true;
    inputs.pivotPositionRads = pivotEncoder.getPosition(); 
    inputs.pivotAppliedVoltage = pivotMotor.getBusVoltage();
    inputs.pivotRPM = pivotEncoder.getVelocity();

  }

  // Roller Methods
  @Override
  public void setRollerVoltage(double targetVolts) {
    rollerMotor.setVoltage(MathUtil.clamp(targetVolts, -12.0, 12.0));
  }

  @Override
  public void setRollerRPM(double targetRPM) {
    double ffVolts = rollerFeedforward.calculate(targetRPM);
    rollerController.setSetpoint(
        targetRPM, SparkFlex.ControlType.kVelocity, ClosedLoopSlot.kSlot0, ffVolts);
  }

  @Override
  public void updateRollerPID(SparkFlexConfig config) {
    this.rollerConfig = config;
    rollerMotor.configure(
        rollerConfig,
        com.revrobotics.ResetMode.kNoResetSafeParameters,
        com.revrobotics.PersistMode.kNoPersistParameters);
  }

  @Override
  public void updateRollerFeedforward(double kS, double kV) {
    this.rollerFeedforward = new SimpleMotorFeedforward(kS, kV);
  }

  // Pivot Methods
  @Override
  public void setPivotVoltage(double volts) {
    this.pivotVolts = MathUtil.clamp(volts, -12.0, 12.0);
    pivotMotor.setVoltage(this.pivotVolts);
  }
  
  @Override
  public void setPivotPosition(double targetPositionRads) {
    pivotSetpoint = pivotProfile.calculate(0.020, pivotSetpoint, new TrapezoidProfile.State(targetPositionRads, 0));
    double ffVolts = pivotFeedforward.calculate(pivotSetpoint.position, pivotSetpoint.velocity);
    pivotController.setSetpoint(pivotSetpoint.position, SparkMax.ControlType.kPosition, ClosedLoopSlot.kSlot0, ffVolts);
  }

  @Override
  public void updatePivotPID(SparkMaxConfig config) {
    this.pivotConfig = config;
    pivotMotor.configure(
        pivotConfig,
        com.revrobotics.ResetMode.kNoResetSafeParameters,
        com.revrobotics.PersistMode.kNoPersistParameters);
  }

  @Override
  public void updatePivotFeedforward(double kS, double kG, double kV) {
    this.pivotFeedforward = new ArmFeedforward(kS, kG, kV);
  }

  @Override
  public void stopMotors() {
    pivotMotor.stopMotor();
    rollerMotor.stopMotor();
  }
  
}
