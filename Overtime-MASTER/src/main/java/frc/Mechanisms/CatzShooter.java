package frc.Mechanisms;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Utils.Conversions;
import frc.Utils.Util;
import frc.robot.Robot;

public class CatzShooter {

    private final WPI_TalonFX topRoller;
    private final WPI_TalonFX btmRoller; 
    private final CANSparkMax feeder;

    //Conversions
    final double COUNTS_PER_REVOLUTION      = 2048.0;  //Falcon Encoder counts per rotation
    final double SEC_PER_MIN                = 60.0;    
    final double ENCODER_SAMPLE_RATE_MSEC   = 100.0;
    final double ENCODER_SAMPLE_PERIOD_MSEC = (1.0 / ENCODER_SAMPLE_RATE_MSEC);
    final double MSEC_TO_SEC                = 1000.0;
    final double HUNDRED_MSEC_PER_SEC       = 10.0;

    //offsets
    public final double SHOOTER_RPM_PID_OFFSET_TOP      = -85.0;
    public final double SHOOTER_RPM_PID_OFFSET_BTM      = -85.0;

    //converts velocity to RPM
    final double CONV_COUNTS_PER_100_MSEC_TO_RPM  = ( ((ENCODER_SAMPLE_PERIOD_MSEC * MSEC_TO_SEC * SEC_PER_MIN) / COUNTS_PER_REVOLUTION) ); 
    final double CONV_RPM_TO_COUNTS_PER_100_MSEC  = (COUNTS_PER_REVOLUTION) * (1.0/SEC_PER_MIN) * (1.0/HUNDRED_MSEC_PER_SEC);

    //setup for kF
    private double PID_kF_TOP                   = (1023.0/20666.0);
    private double PID_kF_BTM                   = (1023.0/20666.0);

    private final static double SHOOT_VEL_HIGH_TOP = 1000; //TBD
    private final static double SHOOT_VEL_HIGH_BOT = 1000;
    
    private final static double SHOOT_VEL_MID_TOP  = 700; //TBD
    private final static double SHOOT_VEL_MID_BOT  = 700; 

    private final static double SHOOT_VEL_CUBE_TRANSFER_TOP = 800; //TBD
    private final static double SHOOT_VEL_CUBE_TRANSFER_BOT = 800;

    private final double kF_TOP_MOTOR = PID_kF_TOP; 
    private final double kP_TOP_MOTOR = 0.00005;
    private final double kI_TOP_MOTOR = 0.0;
    private final double kD_TOP_MOTOR = 0.0;

    private final double kF_BOT_MOTOR = PID_kF_BTM;
    private final double kP_BOT_MOTOR = 0.00005;
    private final double kI_BOT_MOTOR = 0.0;
    private final double kD_BOT_MOTOR = 0.0;

    private final int TOP_ROLLER_CAN_ID = 11; //see component map
    private final int BTM_ROLLER_CAN_ID  = 10;
    private final int FEEDER_ROLLER_CAN_ID = 12;

    private final SupplyCurrentLimitConfiguration currentLimit;

    private final double  CURRENT_LIMIT_AMPS            = 60.0;
    private final double  CURRENT_LIMIT_TRIGGER_AMPS    = 60.0; //TBD Should be 55 current limited?
    private final double  CURRENT_LIMIT_TIMEOUT_SECONDS = 0.5;
    private final boolean ENABLE_CURRENT_LIMIT          = true;

    private final int FEEDER_MC_CURRENT_LIMIT        = 60;

    private final double FEEDER_SHOOT_SPEED  = 0.7;
    private final double FEEDER_INTAKE_SPEED = 0.15;

    

    private enum ShooterState
    {
        OFF,
        WAIT_FOR_STEADY,
        READY,
        SHOOTING;
    }

    public enum ShootingMode
    {
        HIGH(SHOOT_VEL_HIGH_TOP, SHOOT_VEL_HIGH_BOT),
        MID(SHOOT_VEL_MID_TOP, SHOOT_VEL_MID_BOT),
        CUBE_TRANSFER(SHOOT_VEL_CUBE_TRANSFER_TOP, SHOOT_VEL_CUBE_TRANSFER_BOT);

        public double shootVelTop;
        public double shootVelBtm;

        private ShootingMode(double shootVelTop, double shootVelBtm)
        {
            this.shootVelTop = shootVelTop;
            this.shootVelBtm = shootVelBtm;
        }
    }

    private ShooterState shooterState = ShooterState.OFF;

    private double topRollerRPM = 0.0;
    private double botRollerRPM = 0.0;

    private volatile double topRollerTargetRPM = 0.0;
    private volatile double botRollerTargetRPM = 0.0;

    private final double SHOOTER_RPM_STEADY_THRESHOLD_SCALING_FACTOR = 0.06;

    private int shooterRPMStableCounter = 0;
    private final int SHOOTER_RPM_STEADY_THRESHOLD = 10; //0.2 second
    
    private int shootingCounter = 0;
    private final int SHOOTING_COUNTER_THRESHOLD = 100; //two seconds

    private boolean rumbleSet = false;

    // private double minThreshHoldTop = 0.0;
    // private double maxThreshHoldTop = 0.0;
    // private double minThreshHoldBot = 0.0;
    // private double maxThreshHoldBot = 0.0;

    public CatzShooter()
    {

        feeder = new CANSparkMax(FEEDER_ROLLER_CAN_ID, MotorType.kBrushless);
        
        feeder.restoreFactoryDefaults();
        feeder.setIdleMode(IdleMode.kCoast);
        feeder.setSmartCurrentLimit(FEEDER_MC_CURRENT_LIMIT);

        topRoller = new WPI_TalonFX(TOP_ROLLER_CAN_ID);
        btmRoller = new WPI_TalonFX(BTM_ROLLER_CAN_ID);

        currentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT, CURRENT_LIMIT_AMPS, CURRENT_LIMIT_TRIGGER_AMPS, CURRENT_LIMIT_TIMEOUT_SECONDS);

        topRoller.configFactoryDefault();
        topRoller.setNeutralMode(NeutralMode.Coast);
        topRoller.configSupplyCurrentLimit(currentLimit);

        topRoller.config_kF(0, kF_TOP_MOTOR);
        topRoller.config_kP(0, kP_TOP_MOTOR);
        topRoller.config_kI(0, kI_TOP_MOTOR);
        topRoller.config_kD(0, kD_TOP_MOTOR);

        btmRoller.configFactoryDefault();
        btmRoller.setNeutralMode(NeutralMode.Coast);
        btmRoller.configSupplyCurrentLimit(currentLimit);

        btmRoller.config_kF(0, kF_BOT_MOTOR);
        btmRoller.config_kP(0, kP_BOT_MOTOR);
        btmRoller.config_kI(0, kI_BOT_MOTOR);
        btmRoller.config_kD(0, kD_BOT_MOTOR);


        startShooterThread();
    }


    
    public void startShooterThread()
    {
        Thread shooterThread = new Thread(()->{
            while(true)
            {
                topRollerRPM = Math.abs(topRoller.getSelectedSensorVelocity() * CONV_COUNTS_PER_100_MSEC_TO_RPM);
                botRollerRPM = Math.abs(btmRoller.getSelectedSensorVelocity() * CONV_COUNTS_PER_100_MSEC_TO_RPM);
        
                switch(shooterState)
                {
                    case OFF:
                    break;
        
                    case WAIT_FOR_STEADY:
                       
                       // System.out.println("TOP: " + minThreshHoldTop + " < " + topRollerRPM + " < " + maxThreshHoldTop);
                       // System.out.println("BOT: " + minThreshHoldBot + " < " + botRollerRPM + " < " + maxThreshHoldBot); 
                                           

                        // ((topRollerRPM >= minThreshHoldTop) && (topRollerRPM <= maxThreshHoldTop)) && 
                        //((botRollerRPM >= minThreshHoldBot) && (botRollerRPM <= maxThreshHoldBot)) 
                        if (Util.epsilonEquals(topRollerRPM, topRollerTargetRPM, topRollerTargetRPM * SHOOTER_RPM_STEADY_THRESHOLD_SCALING_FACTOR) && Util.epsilonEquals(botRollerRPM, botRollerTargetRPM, botRollerTargetRPM * SHOOTER_RPM_STEADY_THRESHOLD_SCALING_FACTOR))
                        {
                            shooterRPMStableCounter++;
        
                            if(shooterRPMStableCounter >= SHOOTER_RPM_STEADY_THRESHOLD)
                            {
                                shooterState = ShooterState.READY;
                                shooterRPMStableCounter = 0;
                            }
                        }
                        else
                        {
                            shooterRPMStableCounter = 0;
                        }
                    break;
        
                    case READY:
                        if(rumbleSet == false)
                        {
                            Robot.xboxAux.setRumble(RumbleType.kLeftRumble, 1.0);
                            rumbleSet = true;
                        }
                    break;
        
                    case SHOOTING:
                        shootingCounter++;
        
                        Robot.indexer.feedCubeToShooter();
                        if(shootingCounter >= SHOOTING_COUNTER_THRESHOLD)
                        {
                            shooterOff();
                            Robot.indexer.indexerIntake();
                        }
                    break;
                }
                Timer.delay(0.02);
            }
        });
    
        shooterThread.start();
    }

    public void cmdProcShooter(boolean topScore, boolean midScore, boolean cubeTransfer, boolean shoot, boolean abort)
    {
        if(topScore)
        {
            topRollerTargetRPM = SHOOT_VEL_HIGH_TOP;
            botRollerTargetRPM = SHOOT_VEL_HIGH_BOT;

            setTargetVelocity();
        }
        else if(midScore)
        {
            topRollerTargetRPM = SHOOT_VEL_MID_TOP;
            botRollerTargetRPM = SHOOT_VEL_MID_BOT;

            setTargetVelocity();
        }
        else if(cubeTransfer)
        {
            topRollerTargetRPM = SHOOT_VEL_CUBE_TRANSFER_TOP;
            botRollerTargetRPM = SHOOT_VEL_CUBE_TRANSFER_BOT;

            setTargetVelocity();
        }
        
        if(shoot)
        {
            shoot();
        }

        if(abort)
        {
            shooterOff();
        }
    }

    public void printTemperatures()
    {
        System.out.println("Top: " + topRoller.getTemperature());
        System.out.println("Btm: " + btmRoller.getTemperature());
    }

    public void shoot()
    {
        shooterState = ShooterState.SHOOTING;
        feeder.set(FEEDER_SHOOT_SPEED);
    }

    private void shooterOff()
    {
        shooterState = ShooterState.OFF;

        shootingCounter = 0;

        Robot.xboxAux.setRumble(RumbleType.kLeftRumble, 0.0);

        topRollerTargetRPM = 0.0;
        botRollerTargetRPM = 0.0;
        
        topRoller.set(ControlMode.PercentOutput, 0.0);
        btmRoller.set(ControlMode.PercentOutput, 0.0);
        feeder.set(0.0);
    }

    private void setTargetVelocity()
    {
        topRollerTargetRPM += SHOOTER_RPM_PID_OFFSET_TOP;
        double velTop = -Conversions.RPMToFalcon(topRollerTargetRPM, 1.0);
        topRoller.set(ControlMode.Velocity, velTop);

        botRollerTargetRPM += SHOOTER_RPM_PID_OFFSET_BTM;
        double velBot = Conversions.RPMToFalcon(botRollerTargetRPM, 1.0);
        btmRoller.set(ControlMode.Velocity, velBot);

        // minThreshHoldTop = topRollerTargetRPM - SHOOTER_RPM_STEADY_RANGE;
        // maxThreshHoldTop = topRollerTargetRPM + SHOOTER_RPM_STEADY_RANGE;
        // minThreshHoldBot = botRollerTargetRPM - SHOOTER_RPM_STEADY_RANGE;
        // maxThreshHoldBot = botRollerTargetRPM + SHOOTER_RPM_STEADY_RANGE;

        rumbleSet = false;
        shooterState = ShooterState.WAIT_FOR_STEADY;       
    }

    /* FOR AUTONOMOUS */
    public void cubeScore(ShootingMode mode, double timeout)
    {
        topRollerTargetRPM = mode.shootVelTop;
        botRollerTargetRPM = mode.shootVelBtm;
        
        setTargetVelocity();

        double startTime = Timer.getFPGATimestamp();
        double elapsedTime = 0.0;
        while(true) //wait for shooter to rev up
        {
            elapsedTime = Timer.getFPGATimestamp() - startTime;
            if(shooterState == ShooterState.READY || elapsedTime >= timeout)
            {
                shoot();
                break;
            }
            Timer.delay(0.01);

        }
    }

    public void revUpShootMotor(ShootingMode mode)
    {
        topRollerTargetRPM = mode.shootVelTop;
        botRollerTargetRPM = mode.shootVelBtm;

        // topRoller.selectProfileSlot(mode.pidSlot, 0);
        // btmRoller.selectProfileSlot(mode.pidSlot, 0);

        setTargetVelocity();
    }

    public boolean finishedShooting()
    {
        return shooterState == ShooterState.OFF;
    }

    public void intakeCube()
    {
        feeder.set(FEEDER_INTAKE_SPEED);
    }

    public void feederOff()
    {
        feeder.set(0.0);
    }

    public void smartdashboardShooter()
    {
        // SmartDashboard.putNumber("top sensor velocity", topRoller.getSelectedSensorVelocity());
        // SmartDashboard.putNumber("btm sensor velocity", btmRoller.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Top Roller RPM", topRollerRPM);
        SmartDashboard.putNumber("Top Roller RPM Graph", topRollerRPM);

        SmartDashboard.putNumber("Btm Roller RPM", botRollerRPM);
        SmartDashboard.putNumber("Btm Roller RPM Graph", botRollerRPM);

        SmartDashboard.putNumber("toproller rotation per 100ms", Math.abs(topRoller.getSelectedSensorVelocity()));
        SmartDashboard.putNumber("bottomroller RPM", botRollerRPM);
    }
}