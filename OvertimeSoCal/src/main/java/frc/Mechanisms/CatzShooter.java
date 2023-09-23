package frc.Mechanisms;

import javax.lang.model.util.ElementScanner14;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.Utils.Conversions;
import frc.Utils.Util;
import frc.robot.Robot;

public class CatzShooter {
    /*
     * 
     * [IMPORTANT]                                      [IMPORTANT]
     * [IMPORTANT]    ALL CONSTANTS ARE DUMMY VALUES    [IMPORTANT]
     * [IMPORTANT]                                      [IMPORTANT]
     * 
     * 
     */

    private final WPI_TalonFX topRoller; //change name to top and bottom //make variable names the same length
    private final WPI_TalonFX btmRoller; 

    private final double topRollerGearRatio = 1.0 / 1.0;
    private final double botRollerGearRatio = 1.0 / 1.0;

    private final double kF_TOP = 0.6; 
    private final double kP_TOP = 1.0; 
    private final double kI_TOP = 0.0;
    private final double kD_TOP = 0.0;

    private final double kF_BOT = 0.6; 
    private final double kP_BOT = 1.0; 
    private final double kI_BOT = 0.0;
    private final double kD_BOT = 0.0;


    private final int TOP_ROLLER_CAN_ID = 999; //see component map
    private final int BOT_ROLLER_CAN_ID  = 999;

    private final SupplyCurrentLimitConfiguration currentLimit;

    private final double  CURRENT_LIMIT_AMPS            = 60.0;
    private final double  CURRENT_LIMIT_TRIGGER_AMPS    = 60.0;
    private final double  CURRENT_LIMIT_TIMEOUT_SECONDS = 0.5;
    private final boolean ENABLE_CURRENT_LIMIT          = true;

    private final double SHOOT_VEL_CUBE_TRANSFER_TOP = 4000; //RPM
    private final double SHOOT_VEL_HIGH_TOP = 2000;
    private final double SHOOT_VEL_MID_TOP = 1000;

    private final double SHOOT_VEL_CUBE_TRANSFER_BOT = 4000; //RPM
    private final double SHOOT_VEL_HIGH_BOT = 2000;
    private final double SHOOT_VEL_MID_BOT = 1000;

    private enum ShooterState
    {
        OFF,
        WAIT_FOR_STEADY,
        READY,
        SHOOTING;
    }

    private ShooterState shooterState = ShooterState.OFF;

    private double topRollerRPM = 0.0;
    private double botRollerRPM = 0.0;

    private double topRollerTargetRPM = 0.0;
    private double botRollerTargetRPM = 0.0;

    private int shooterRPMStableCounter = 0;
    private final int SHOOTER_RPM_STEADY_THRESHOLD = 10; //0.1 second

    private int shootingCounter = 0;
    private final int SHOOTING_COUNTER_THRESHOLD = 200; //two seconds

    private final double SHOOTER_RPM_STEADY_RANGE = 50.0;

    private final double MOTOR_RPM_CONVERSION_FACTOR = 10.0 * 60.0 / 2048.0;


    boolean rumbleSet = false;

    public CatzShooter()
    {
        topRoller = new WPI_TalonFX(TOP_ROLLER_CAN_ID);
        btmRoller = new WPI_TalonFX(BOT_ROLLER_CAN_ID);

        currentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT, CURRENT_LIMIT_AMPS, CURRENT_LIMIT_TRIGGER_AMPS, CURRENT_LIMIT_TIMEOUT_SECONDS);

        topRoller.configFactoryDefault();
        topRoller.setNeutralMode(NeutralMode.Coast);
        topRoller.configSupplyCurrentLimit(currentLimit);

        topRoller.config_kF(0, kF_TOP);
        topRoller.config_kP(0, kP_TOP);
        topRoller.config_kI(0, kI_TOP);
        topRoller.config_kD(0, kD_TOP);


        btmRoller.configFactoryDefault();
        btmRoller.setNeutralMode(NeutralMode.Coast);
        btmRoller.configSupplyCurrentLimit(currentLimit);

        btmRoller.config_kF(0, kF_BOT);
        btmRoller.config_kP(0, kP_BOT);
        btmRoller.config_kI(0, kI_BOT);
        btmRoller.config_kD(0, kD_BOT);
    }

    public void shooterPeriodicUpdate()
    {
        topRollerRPM = topRoller.getSelectedSensorVelocity() * MOTOR_RPM_CONVERSION_FACTOR;
        botRollerRPM = btmRoller.getSelectedSensorVelocity() * MOTOR_RPM_CONVERSION_FACTOR;

                switch(shooterState)
                {
                    case OFF:
                        if(topRollerTargetRPM > 0.0)
                        {
                            shooterState = ShooterState.WAIT_FOR_STEADY;

                            setTargetVelocity();
                            rumbleSet = false;
                        }
                    break;

                    case WAIT_FOR_STEADY:
                        if(Util.epsilonEquals(topRollerRPM, topRollerTargetRPM, SHOOTER_RPM_STEADY_RANGE) && Util.epsilonEquals(botRollerRPM, botRollerTargetRPM, SHOOTER_RPM_STEADY_RANGE))
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
                        if(!rumbleSet)
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
                            Robot.indexer.indexerOff();
                            shooterOff(); //TBD make a button to abort
                        }
                    break;
                }
    }

    public void cmdProcShooter(boolean topScore, boolean midScore, boolean cubeTransfer, boolean shoot)
    {
        if(topScore)
        {
            topRollerTargetRPM = SHOOT_VEL_HIGH_TOP;
            botRollerTargetRPM = SHOOT_VEL_HIGH_BOT;
        }
        else if(midScore)
        {
            topRollerTargetRPM = SHOOT_VEL_MID_TOP;
            botRollerTargetRPM = SHOOT_VEL_MID_BOT;
        }
        else if(cubeTransfer)
        {
            topRollerTargetRPM = SHOOT_VEL_CUBE_TRANSFER_TOP;
            botRollerTargetRPM = SHOOT_VEL_CUBE_TRANSFER_BOT;
        }
        
        else if(shoot)
        {
            shoot();
        }
    }

    private void shoot()
    {
        shooterState = ShooterState.SHOOTING;
    }

    private void shooterOff()
    {
        shooterState = ShooterState.OFF;

        shootingCounter = 0;

        Robot.xboxAux.setRumble(RumbleType.kLeftRumble, 0.0);

        topRollerTargetRPM = 0.0;
        botRollerTargetRPM = 0.0;
        
        //turn pid off

        topRoller.set(0.0);
        btmRoller.set(0.0);
    }

    private void setTargetVelocity()
    {
        double velTop = Conversions.RPMToFalcon(topRollerRPM, topRollerGearRatio);
        topRoller.set(ControlMode.Velocity, velTop);

        double velBot = Conversions.RPMToFalcon(botRollerRPM, botRollerGearRatio);
        btmRoller.set(ControlMode.Velocity, velBot);
    }

    
}
