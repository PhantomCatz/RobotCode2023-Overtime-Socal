package frc.Mechanisms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;

public class CatzIndexer {

    private WPI_TalonFX ltIndexerMotor;
    private WPI_TalonFX rtIndexerMotor;

    private final int INDEXER_MOTOR_CAN_ID_LT = 4; //name should be left-canID
    private final int INDEXER_MOTOR_CAN_ID_RT = 4; //see component map

    private final SupplyCurrentLimitConfiguration currentLimit;

    private final double  CURRENT_LIMIT_AMPS            = 60.0;
    private final double  CURRENT_LIMIT_TRIGGER_AMPS    = 60.0;
    private final double  CURRENT_LIMIT_TIMEOUT_SECONDS = 0.5;
    private final boolean ENABLE_CURRENT_LIMIT          = true;

    private final double INDEXER_MOTOR_POWER_SHOOT = 1.0;
    private final double INDEXER_MOTOR_POWER_INT    =  0.5; 
    private final double INDEXER_MOTOR_POWER_OFF   =  0.0;
    private final double INDEXER_MOTOR_POWER_OUT   = -0.5;

    private DigitalInput indexerBeamBreak;

    private final int INDEXER_BEAM_BREAK_DIO_PORT = 2637;

    public CatzIndexer() 
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

    //False means the beam broke and true means the beams are connected??
    public boolean getBeamBreak()
    {
        return indexerBeamBreak.get();
    }

    public void indexerOff()
    {
        runIndexer(INDEXER_MOTOR_POWER_OFF);
    }

    public void feedCubeToShooter()
    {
        runIndexer(INDEXER_MOTOR_POWER_SHOOT);
    }

    public void indexerIntake()
    {
        runIndexer(INDEXER_MOTOR_POWER_INT);
    }

    public void indexerOuttake()
    {
        runIndexer(INDEXER_MOTOR_POWER_OUT);
    }

    private void runIndexer(double power)
    {
        rtIndexerMotor.set(ControlMode.PercentOutput, power);
        ltIndexerMotor.set(ControlMode.PercentOutput, power);
    }
}