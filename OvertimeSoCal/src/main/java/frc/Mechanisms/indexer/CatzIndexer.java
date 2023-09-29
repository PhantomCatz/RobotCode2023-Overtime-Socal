package frc.Mechanisms.indexer;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.Utils.CatzConstants;

public class CatzIndexer {
    private static CatzIndexer instance = null;
    private final IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    //returns itself for singleton implementation
    public static CatzIndexer getInstance()
    {
        if(instance == null)
        {
            instance = new CatzIndexer();
        }

        return instance;
    }


    private final double INDEXER_MOTOR_POWER_SHOOT = 1.0;
    private final double INDEXER_MOTOR_POWER_INT    =  0.5; 
    private final double INDEXER_MOTOR_POWER_OFF   =  0.0;
    private final double INDEXER_MOTOR_POWER_OUT   = -0.5;


    private CatzIndexer() 
    {
        switch(CatzConstants.currentMode)
        {
            case REAL:
                io = new IndexerReal();
                break;
            case SIM :
                io = null;
                break;
            default:
                io = new IndexerReal() {};
                break;
        }
    }

    public void indexerPeriodic()
    {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("indexer", inputs);
    }
    //False means the beam broke and true means the beams are connected??
    public boolean getBeamBreakOpen()
    {
        return inputs.indexerBeamBreakOpen;
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
        runIndexer(power);
    }
}