package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.VoltageUnit;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO.FlywheelIOInputs;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs() {
        // Rollers
        public boolean rollerConnected = false;
        public double rollerVelocityRadsPerSec = 0.0;
        public double rollerAppliedVoltage = 0.0;

        // Pivot
        public boolean pivotConnected = false;
        public double pivotPositionRads = 0.0;
        public double pivotVelocityRadsPerSec = 0.0;
        public double pivotAppliedVoltage = 0.0;
    }

    public default void updateInputs(FlywheelIOInputs inputs) {}

    public default void setRollerVelocity(double rollerVelocityRadsPerSec, VoltageUnit feedforwardVolts) {}

    public void setRollerVoltage(double volts){}

    public void setPivotPosition(double ){}

    public default void setPivotVelocity(double rollerVelocityRadsPerSec, VoltageUnit feedforwardVolts) {}

    public void setPivotVoltage(double volts){}

}
