package frc.Mechanisms.shooter;

import javax.lang.model.util.ElementScanner14;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Utils.CatzConstants;
import frc.Utils.Conversions;
import frc.Utils.Util;
import frc.robot.Robot;

public class CatzShooter {

    private static CatzShooter instance = null;
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    //returns itself for singleton implementation
    public static CatzShooter getIntstance()
    {
        if(instance == null)
        {
            instance = new CatzShooter();
        }

        return instance;
    }

    private enum ShooterState
    {
        OFF,
        WAIT_FOR_STEADY,
        READY,
        SHOOTING;
    }

    private ShooterState shooterState = ShooterState.OFF;


    private final double topRollerGearRatio = 2.0 / 1.0;
    private final double botRollerGearRatio = 2.0 / 1.0;

    private double topRollerRPM = 0.0;
    private double botRollerRPM = 0.0;

    private volatile double topRollerTargetRPM = 0.0;
    private volatile double botRollerTargetRPM = 0.0;

    private int shooterRPMStableCounter = 0;
    private final int SHOOTER_RPM_STEADY_THRESHOLD = 10; //0.1 second

    private int shootingCounter = 0;
    private final int SHOOTING_COUNTER_THRESHOLD = 200; //two seconds

    private final double SHOOTER_RPM_STEADY_RANGE = 50.0;

    private final double MOTOR_RPM_CONVERSION_FACTOR = 10.0 * 60.0 / 2048.0;

    private final double SHOOT_VEL_CUBE_TRANSFER_TOP = 10; //RPM
    private final double SHOOT_VEL_HIGH_TOP = 50;
    private final double SHOOT_VEL_MID_TOP = 45;

    private final double SHOOT_VEL_CUBE_TRANSFER_BOT = 10; //RPM
    private final double SHOOT_VEL_HIGH_BOT = 50;
    private final double SHOOT_VEL_MID_BOT = 45;


    boolean rumbleSet = false;

    private CatzShooter()
    {
        switch(CatzConstants.currentMode)
        {
            case REAL:
                io = new ShooterIOReal();
                break;
            case SIM :
                io = null;
                break;
            default:
                io = new ShooterIOReal() {};
                break;
        }
    }
    public void shooterPeriodic()
    {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Shooter", inputs);
    }

    public void shooterPeriodicUpdate()
    {
        topRollerRPM = inputs.topRollerSelectedSensorVelocity * MOTOR_RPM_CONVERSION_FACTOR;
        botRollerRPM = inputs.btmRollerSelectedSensorVelocity * MOTOR_RPM_CONVERSION_FACTOR;

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
                                System.out.println("ready");
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
                            System.out.println("rumblin'");
                            Robot.xboxAux.setRumble(RumbleType.kLeftRumble, 1.0);
                            rumbleSet = true;
                        }
                    break;

                    case SHOOTING:
                        shootingCounter++;

                        //Robot.indexer.feedCubeToShooter(); TBD UNCOMMENT

                        if(shootingCounter >= SHOOTING_COUNTER_THRESHOLD)
                        {
                            //Robot.indexer.indexerOff(); TBD uncomment
                            shooterOff(); //TBD make a button to abort
                        }
                    break;
                }

    }

    

    /**
     * parameters: topscore, midscore, cubetransfer, shoot
     **/
    public void cmdProcShooter(boolean topScore, boolean midScore, boolean cubeTransfer, boolean shoot, boolean abort)
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
        System.out.println("Top: " + inputs.topRollerTemp);
        System.out.println("Btm: " + inputs.btmRollerTemp);
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

        io.setRollersOff();
    }

    private void setTargetVelocity()
    {
        double velTop = -Conversions.RPMToFalcon(topRollerTargetRPM, topRollerGearRatio);
        io.setTopRollerVelocity(velTop);

        double velBot = Conversions.RPMToFalcon(botRollerTargetRPM, botRollerGearRatio);
        io.setBtmRollerVelocity(velBot);
        System.out.println(velTop);
    }

    public void smartdashboardShooter()
    {
        SmartDashboard.putNumber("top sensor velocity", inputs.topRollerSelectedSensorVelocity);
        SmartDashboard.putNumber("btm sensor velocity", inputs.btmRollerSelectedSensorVelocity);
        SmartDashboard.putNumber("toproller RPM", topRollerRPM);
        SmartDashboard.putNumber("bottomroller RPM", botRollerRPM);
        
    }

    
}
