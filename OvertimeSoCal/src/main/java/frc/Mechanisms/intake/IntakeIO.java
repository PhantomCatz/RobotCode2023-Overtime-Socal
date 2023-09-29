package frc.Mechanisms.intake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public interface IntakeIO 
{
    @AutoLog
    public class IntakeIOInputs
    {
        public Value intakeSolenoidValue;

    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setIntakeSolenoidIO(Value value) {}

    public default void setIntakeRollersPercentIO(double pwr) {}
}
