package frc.Mechanisms.indexer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class IndexerReal implements IndexerIO
{
    private WPI_TalonFX ltIndexerMotor;
    private WPI_TalonFX rtIndexerMotor;

    private final int INDEXER_MOTOR_CAN_ID_LT = 4; //name should be left-canID
    private final int INDEXER_MOTOR_CAN_ID_RT = 4; //see component map

    private final SupplyCurrentLimitConfiguration currentLimit;

    private final double  CURRENT_LIMIT_AMPS            = 60.0;
    private final double  CURRENT_LIMIT_TRIGGER_AMPS    = 60.0;
    private final double  CURRENT_LIMIT_TIMEOUT_SECONDS = 0.5;
    private final boolean ENABLE_CURRENT_LIMIT          = true;

    private DigitalInput indexerBeamBreak;

    private final int INDEXER_BEAM_BREAK_DIO_PORT = 2637;
    
    public IndexerReal()
    {
        ltIndexerMotor = new WPI_TalonFX(INDEXER_MOTOR_CAN_ID_LT);
        rtIndexerMotor = new WPI_TalonFX(INDEXER_MOTOR_CAN_ID_RT);

        currentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT, CURRENT_LIMIT_AMPS, CURRENT_LIMIT_TRIGGER_AMPS, CURRENT_LIMIT_TIMEOUT_SECONDS);

        ltIndexerMotor.configFactoryDefault();
        ltIndexerMotor.setNeutralMode(NeutralMode.Brake); //change to brake
        ltIndexerMotor.configSupplyCurrentLimit(currentLimit);

        rtIndexerMotor.configFactoryDefault();
        rtIndexerMotor.setNeutralMode(NeutralMode.Brake); //change to brake
        rtIndexerMotor.configSupplyCurrentLimit(currentLimit);
        
        rtIndexerMotor.setInverted(true);
        rtIndexerMotor.follow(ltIndexerMotor);

        //indexerBeamBreak = new DigitalInput(INDEXER_BEAM_BREAK_DIO_PORT); TBD UNComment
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) 
    {
        inputs.indexerBeamBreakOpen = indexerBeamBreak.get();
    }

    @Override
    public void runIndexerPercentIO(double pwr)
    {
        rtIndexerMotor.set(ControlMode.PercentOutput, pwr);
        ltIndexerMotor.set(ControlMode.PercentOutput, pwr);
    }



}
