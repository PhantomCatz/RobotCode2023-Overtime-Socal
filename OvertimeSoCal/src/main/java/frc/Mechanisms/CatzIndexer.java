package frc.Mechanisms;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

import frc.robot.Robot;


public class CatzIndexer {

    private CANSparkMax ltIndexerMotor;
    private CANSparkMax rtIndexerMotor;


    private final int INDEXER_MOTOR_CAN_ID_LT = 20; 
    private final int INDEXER_MOTOR_CAN_ID_RT = 21; 

    private final SupplyCurrentLimitConfiguration currentLimit;
    private final int INDEXER_MC_CURRENT_LIMIT = 60;

    private final double  CURRENT_LIMIT_AMPS            = 60.0;
    private final double  CURRENT_LIMIT_TRIGGER_AMPS    = 60.0;
    private final double  CURRENT_LIMIT_TIMEOUT_SECONDS = 0.5;
    private final boolean ENABLE_CURRENT_LIMIT          = true;

    private final double INDEXER_MOTOR_POWER_SHOOT = 1.0;
    private final double INDEXER_MOTOR_POWER_INT    =  0.5; 
    private final double INDEXER_MOTOR_POWER_OFF   =  0.0;
    private final double INDEXER_MOTOR_POWER_OUT   = -0.5;

    private DigitalInput indexerBeamBreak;

    boolean beamBreakBroken = false;
    boolean intakeStage = false;

    private final int INDEXER_BEAM_BREAK_DIO_PORT = 5;

    Timer indexerTimer = new Timer();

    private Thread Indexer;

    private double indexerTime;
    public CatzIndexer() 
    {
        ltIndexerMotor = new CANSparkMax(INDEXER_MOTOR_CAN_ID_LT, MotorType.kBrushless);
        rtIndexerMotor = new CANSparkMax(INDEXER_MOTOR_CAN_ID_RT, MotorType.kBrushless);

        currentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT, CURRENT_LIMIT_AMPS, CURRENT_LIMIT_TRIGGER_AMPS, CURRENT_LIMIT_TIMEOUT_SECONDS);

        ltIndexerMotor.restoreFactoryDefaults();
        ltIndexerMotor.setIdleMode(IdleMode.kBrake); //change to brake
        ltIndexerMotor.setSmartCurrentLimit(INDEXER_MC_CURRENT_LIMIT);

        rtIndexerMotor.restoreFactoryDefaults();
        rtIndexerMotor.setIdleMode(IdleMode.kBrake); //change to brake
        rtIndexerMotor.setSmartCurrentLimit(INDEXER_MC_CURRENT_LIMIT);
        
        rtIndexerMotor.setInverted(true);

        indexerBeamBreak = new DigitalInput(INDEXER_BEAM_BREAK_DIO_PORT);

        indexerTimer = new Timer();
        // Indexer = new Thread();

        startIndexer();
    }


    // Indexer Thread 

    public static boolean beamBroken = false;
    
    public void startIndexer()
    {
        // indexerTime = indexerTimer.get();
        // indexerTimer.reset();
        // indexerTimer.start();
        if(Robot.intake.intakeActive == true) 
        {
            System.out.println("Intake Indexer Mode Active");
            // intakeStage = true;
            
            indexerIntake();
        }

        if(getBeamBreak() == true) //&& intakeStage == true
        {
            System.out.println("**********BeamBreakBroken!!");
            // intakeStage = false;
            beamBroken = true;
            indexerOff();
        } else {
            beamBroken = false;
            System.out.println("*************NotBroekn");
        }

        // Indexer = new Thread();
        // {
        //     while(true) 
        //     {
        //       

        //     }
        // }
    }
    
    public boolean getBeamBroken() 
    {
        return beamBroken;
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
        ltIndexerMotor.set(-power);
        rtIndexerMotor.set(-power);
    }

    public void cmdProcIndex(boolean indexRollIn, boolean indexRollOut, boolean indexStop) 
    {
        if(indexRollIn) 
        {   
            indexerIntake();
            Robot.xboxAux.setRumble(RumbleType.kLeftRumble, 1.0);

        } else if (indexRollOut) 
        {
            indexerOuttake();
            Robot.xboxAux.setRumble(RumbleType.kRightRumble, 1.0);

        } 
        
        if (indexStop) 
        {
            indexerOff();
            Robot.xboxAux.setRumble(RumbleType.kLeftRumble, 0.0);
            Robot.xboxAux.setRumble(RumbleType.kRightRumble, 0.0);

        }
    }
}