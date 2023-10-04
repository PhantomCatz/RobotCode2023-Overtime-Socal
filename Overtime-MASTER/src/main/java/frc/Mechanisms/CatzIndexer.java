package frc.Mechanisms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;



public class CatzIndexer {

    private CANSparkMax indexerMotorLt;
    private CANSparkMax indexerMotorRt;


    private final int INDEXER_MOTOR_CAN_ID_LT = 20; 
    private final int INDEXER_MOTOR_CAN_ID_RT = 21; 

    private final int INDEXER_MC_CURRENT_LIMIT = 60;

    private final double INDEXER_MOTOR_POWER_SHOOT  = -0.5;
    private final double INDEXER_MOTOR_POWER_INT    = -0.25; //0.5
    private final double INDEXER_MOTOR_POWER_OUT    =  0.25; 
    private final double INDEXER_MOTOR_POWER_OFF    =  0.0;

    private DigitalInput indexerBeamBreakDIO;

    private final int     INDEXER_BEAM_BREAK_DIO_PORT = 5;
    private final boolean INDEXER_BEAM_CUBE_PRESENT_STATE  = false;
    private final boolean INDEXER_BEAM_NO_CUBE_STATE       = true;

    private Thread monitorBeamBreakThread;

    private final double   INDEXER_MONITOR_BEAM_BREAK_THREAD_PERIOD_SEC = 0.020;

    Timer indexerTimer = new Timer();

    double indexerTime;


    public CatzIndexer() 
    {
        //Neo 550s
        indexerMotorLt = new CANSparkMax(INDEXER_MOTOR_CAN_ID_LT, MotorType.kBrushless);
        indexerMotorRt = new CANSparkMax(INDEXER_MOTOR_CAN_ID_RT, MotorType.kBrushless);

        indexerMotorRt.restoreFactoryDefaults();
        indexerMotorRt.setIdleMode(IdleMode.kBrake); //change to brake
        indexerMotorRt.setSmartCurrentLimit(INDEXER_MC_CURRENT_LIMIT);

        indexerMotorLt.restoreFactoryDefaults();
        indexerMotorLt.setIdleMode(IdleMode.kBrake); //change to brake
        indexerMotorLt.setSmartCurrentLimit(INDEXER_MC_CURRENT_LIMIT);

        indexerMotorLt.setInverted(true);

        //Beam Break
        indexerBeamBreakDIO = new DigitalInput(INDEXER_BEAM_BREAK_DIO_PORT);
        
        monitorBeamBreakSensor();
    }
    



    /*-----------------------------------------------------------------------------------------
    *  
    *  monitorBeamBreakSensor()
    *
    *----------------------------------------------------------------------------------------*/   
    public void monitorBeamBreakSensor()
    {
        monitorBeamBreakThread = new Thread( () ->
        {
            while(true) 
            {
                if(indexerBeamBreakDIO.get() == INDEXER_BEAM_CUBE_PRESENT_STATE)
                {
                    indexerOff();
                    Robot.shooter.feederOff();
                }
                Timer.delay(INDEXER_MONITOR_BEAM_BREAK_THREAD_PERIOD_SEC);
            }

        });   //end of Thread

        monitorBeamBreakThread.start();

    }   //end of monitorBeamBreakSensor()


    public boolean indexerBeamBreakFeedback()
    {
        return indexerBeamBreakDIO.get();
    }


    public void beamBreakBroken()
    {
        if(indexerBeamBreakFeedback() == false) 
        {
            indexerOff();
        }
    }
    
    /*-----------------------------------------------------------------------------------------
    *  
    *  
    *
    *----------------------------------------------------------------------------------------*/   
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
        indexerMotorLt.set(power);
        indexerMotorRt.set(power);
    }

    public void cmdProcIndexer(boolean manualIndexerInward, boolean manualIndexerOuwward, boolean stop)
    {
        if (manualIndexerInward)
        {
            runIndexer(INDEXER_MOTOR_POWER_INT);
        }
        else if (manualIndexerOuwward)
        {
            runIndexer(INDEXER_MOTOR_POWER_OUT);
        }
        else if(stop) 
        {
            indexerOff();
        }
    }

    public void smartdashboardIndexer()
    {
        SmartDashboard.putBoolean("BeamBreakDetect", indexerBeamBreakDIO.get());
    }

}