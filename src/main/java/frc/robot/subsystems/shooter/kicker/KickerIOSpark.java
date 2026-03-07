package frc.robot.subsystems.shooter.kicker;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;

public class KickerIOSpark implements KickerIO {
  private SparkFlex kickerMotor;
  private SparkFlexConfig kickerConfig = new SparkFlexConfig();
  ;
  private RelativeEncoder kickerEncoder;

  public KickerIOSpark() {
    kickerMotor = new SparkFlex(31, MotorType.kBrushless);
    kickerEncoder = kickerMotor.getEncoder();

    kickerConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12.0);

    kickerConfig
        .encoder
        .positionConversionFactor(2 * Math.PI)
        .velocityConversionFactor(2 * Math.PI);

    kickerMotor.configure(
        kickerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(KickerIOInputs inputs) {
    inputs.kickerRPM = kickerEncoder.getVelocity();
  }

  @Override
  public void setKickerVoltage(double targetVolts) {
    kickerMotor.setVoltage(MathUtil.clamp(targetVolts, -12.0, 12.0));
  }

  @Override
  public void stopMotor() {
    kickerMotor.stopMotor();
  }
}
