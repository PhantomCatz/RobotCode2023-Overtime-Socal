package frc.Mechanisms.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class IntakeReal implements IntakeIO
{
    private WPI_TalonFX intakeRoller;
    private final PneumaticsModuleType PCM_TYPE = PneumaticsModuleType.CTREPCM;
    private DoubleSolenoid intakeSolenoid;

    private final int INTAKE_ROLLER_CAN_ID = 2637; //TBD

    private final SupplyCurrentLimitConfiguration currentLimit;

    private final double  CURRENT_LIMIT_AMPS            = 60.0;
    private final double  CURRENT_LIMIT_TRIGGER_AMPS    = 60.0;
    private final double  CURRENT_LIMIT_TIMEOUT_SECONDS = 0.5;
    private final boolean ENABLE_CURRENT_LIMIT          = true;

    private final int GROUND_PCM_PORT    = 2;
    private final int STOWED_PCM_PORT   = 5;

    public IntakeReal()
    {
        intakeRoller = new WPI_TalonFX(INTAKE_ROLLER_CAN_ID);

        currentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT, CURRENT_LIMIT_AMPS, CURRENT_LIMIT_TRIGGER_AMPS, CURRENT_LIMIT_TIMEOUT_SECONDS);

        intakeRoller.configFactoryDefault();
        intakeRoller.setNeutralMode(NeutralMode.Coast);
        intakeRoller.configSupplyCurrentLimit(currentLimit);

        intakeSolenoid = new DoubleSolenoid(PCM_TYPE, GROUND_PCM_PORT, STOWED_PCM_PORT);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs)
    {
        inputs.intakeSolenoidValue = intakeSolenoid.get();
    }

    @Override
    public void setIntakeSolenoidIO(Value value)
    {
        intakeSolenoid.set(value);
    }

    @Override
    public void setIntakeRollersPercentIO(double pwr)
    {
        intakeRoller.set(ControlMode.PercentOutput, pwr);
    }
}
