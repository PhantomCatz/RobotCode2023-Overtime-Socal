package frc.Mechanisms;

import java.util.HashMap;
import java.util.Map;

import javax.xml.transform.Source;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private final CANSparkMax feeder;

    private final double SHOOT_VEL_HIGH_TOP = 1000;
    private final double SHOOT_VEL_HIGH_BOT = 1000;
    
    private final double SHOOT_VEL_MID_TOP  = 700;
    private final double SHOOT_VEL_MID_BOT  = 700; //600 rpm; kF * 0.75
                                                    //1000 rpm; kF * 0.48

    private final double SHOOT_VEL_CUBE_TRANSFER_TOP = 800;
    private final double SHOOT_VEL_CUBE_TRANSFER_BOT = 800;

    private final double kF_TOP_TOP = 0.048;
    private final double kP_TOP_TOP = 0.0;
    private final double kI_TOP_TOP = 0.0;
    private final double kD_TOP_TOP = 0.0;

    private final double kF_BOT_TOP = 0.048; 
    private final double kP_BOT_TOP = 0.0; 
    private final double kI_BOT_TOP = 0.0;
    private final double kD_BOT_TOP = 0.0;

    private final double kF_TOP_MID = 0.048;
    private final double kP_TOP_MID = 0.0;
    private final double kI_TOP_MID = 0.0;
    private final double kD_TOP_MID = 0.0;

    private final double kF_BOT_MID = 0.048;
    private final double kP_BOT_MID = 0.0;
    private final double kI_BOT_MID = 0.0;
    private final double kD_BOT_MID = 0.0;

    private final double kF_TOP_CUBE = 0.048; 
    private final double kP_TOP_CUBE = 0.0; 
    private final double kI_TOP_CUBE = 0.0;
    private final double kD_TOP_CUBE = 0.0;

    private final double kF_BOT_CUBE = 0.048; 
    private final double kP_BOT_CUBE = 0.0; 
    private final double kI_BOT_CUBE = 0.0;
    private final double kD_BOT_CUBE = 0.0;


    private final int TOP_PID_SLOT = 0;
    private final int MID_PID_SLOT = 1;
    private final int CUBE_PID_SLOT = 2;

    private final int TOP_ROLLER_CAN_ID = 11; //see component map
    private final int BTM_ROLLER_CAN_ID  = 10;
    private final int FEEDER_ROLLER_CAN_ID = 12;

    private final SupplyCurrentLimitConfiguration currentLimit;

    private final double  CURRENT_LIMIT_AMPS            = 60.0;
    private final double  CURRENT_LIMIT_TRIGGER_AMPS    = 60.0; //TBD Should be 55 current limited?
    private final double  CURRENT_LIMIT_TIMEOUT_SECONDS = 0.5;
    private final boolean ENABLE_CURRENT_LIMIT          = true;

    private final double FEEDER_SPEED = 0.5;

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

    private volatile double topRollerTargetRPM = 0.0;
    private volatile double botRollerTargetRPM = 0.0;

    private int shooterRPMStableCounter = 0;
    private final int SHOOTER_RPM_STEADY_THRESHOLD = 10; //0.1 second

    private int shootingCounter = 0;
    private final int SHOOTING_COUNTER_THRESHOLD = 100; //two seconds

    private final double SHOOTER_RPM_STEADY_RANGE = 50.0;

    private final double MOTOR_RPM_CONVERSION_FACTOR = 10.0 * 60.0 / 2048.0;

    boolean rumbleSet = false;

    private Map<String, Double[]> stateMapTop = new HashMap<>();
    private String[] targetStatesTop;
    private Map<String, Double[]> stateMapBtm = new HashMap<>();
    private String[] targetStatesBtm;

    public CatzShooter()
    {
        topRoller = new WPI_TalonFX(TOP_ROLLER_CAN_ID);
        btmRoller = new WPI_TalonFX(BTM_ROLLER_CAN_ID);

        feeder = new CANSparkMax(FEEDER_ROLLER_CAN_ID, MotorType.kBrushless);

        //first 4 are FPID, last is target speed
        // stateMapTop.put("Top", new Double[] {0.6, 0.1, 0.0, 0.0, 0.0});
        // stateMapTop.put("Mid", new Double[] {0.6, 0.1, 0.0, 0.0, 0.0});
        // stateMapTop.put("Cube", new Double[] {0.6, 0.1, 0.0, 0.0, 0.0});
        // targetStatesTop = (String[]) stateMapTop.keySet().toArray();

        // stateMapBtm.put("Top", new Double[] {0.6, 0.1, 0.0, 0.0, 0.0});
        // stateMapBtm.put("Mid", new Double[] {0.6, 0.1, 0.0, 0.0, 0.0});
        // stateMapBtm.put("Cube", new Double[] {0.6, 0.1, 0.0, 0.0, 0.0});
        // targetStatesBtm = (String[]) stateMapTop.keySet().toArray();

        // for(int i=0; i < targetStatesTop.length; i++){
        //     topRoller.config_kF(i, stateMapTop.get(targetStatesTop[i])[0]);
        //     topRoller.config_kP(i, stateMapTop.get(targetStatesTop[i])[1]);
        //     topRoller.config_kI(i, stateMapTop.get(targetStatesTop[i])[2]);
        //     topRoller.config_kD(i, stateMapTop.get(targetStatesTop[i])[3]);
        // }

        // for(int i=0; i < targetStatesTop.length; i++){
        //     btmRoller.config_kF(i, stateMapTop.get(targetStatesTop[i])[0]);
        //     btmRoller.config_kP(i, stateMapTop.get(targetStatesTop[i])[1]);
        //     btmRoller.config_kI(i, stateMapTop.get(targetStatesTop[i])[2]);
        //     btmRoller.config_kD(i, stateMapTop.get(targetStatesTop[i])[3]);
        // }

        currentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT, CURRENT_LIMIT_AMPS, CURRENT_LIMIT_TRIGGER_AMPS, CURRENT_LIMIT_TIMEOUT_SECONDS);

        topRoller.configFactoryDefault();
        topRoller.setNeutralMode(NeutralMode.Coast);
        topRoller.configSupplyCurrentLimit(currentLimit);

        topRoller.config_kF(TOP_PID_SLOT, kF_TOP_TOP);
        topRoller.config_kP(TOP_PID_SLOT, kP_TOP_TOP);
        topRoller.config_kI(TOP_PID_SLOT, kI_TOP_TOP);
        topRoller.config_kD(TOP_PID_SLOT, kD_TOP_TOP);

        topRoller.config_kF(MID_PID_SLOT, kF_TOP_MID);
        topRoller.config_kP(MID_PID_SLOT, kP_TOP_MID);
        topRoller.config_kI(MID_PID_SLOT, kI_TOP_MID);
        topRoller.config_kD(MID_PID_SLOT, kD_TOP_MID);

        topRoller.config_kF(CUBE_PID_SLOT, kF_TOP_CUBE);
        topRoller.config_kP(CUBE_PID_SLOT, kP_TOP_CUBE);
        topRoller.config_kI(CUBE_PID_SLOT, kI_TOP_CUBE);
        topRoller.config_kD(CUBE_PID_SLOT, kD_TOP_CUBE);

        btmRoller.configFactoryDefault();
        btmRoller.setNeutralMode(NeutralMode.Coast);
        btmRoller.configSupplyCurrentLimit(currentLimit);

        btmRoller.config_kF(TOP_PID_SLOT, kF_BOT_TOP);
        btmRoller.config_kP(TOP_PID_SLOT, kP_BOT_TOP);
        btmRoller.config_kI(TOP_PID_SLOT, kI_BOT_TOP);
        btmRoller.config_kD(TOP_PID_SLOT, kD_BOT_TOP);

        btmRoller.config_kF(MID_PID_SLOT, kF_BOT_MID);
        btmRoller.config_kP(MID_PID_SLOT, kP_BOT_MID);
        btmRoller.config_kI(MID_PID_SLOT, kI_BOT_MID);
        btmRoller.config_kD(MID_PID_SLOT, kD_BOT_MID);

        btmRoller.config_kF(CUBE_PID_SLOT, kF_BOT_CUBE);
        btmRoller.config_kP(CUBE_PID_SLOT, kP_BOT_CUBE);
        btmRoller.config_kI(CUBE_PID_SLOT, kI_BOT_CUBE);
        btmRoller.config_kD(CUBE_PID_SLOT, kD_BOT_CUBE);
    }

    public void shooterPeriodicUpdate()
    {
        topRollerRPM = Math.abs(topRoller.getSelectedSensorVelocity() * MOTOR_RPM_CONVERSION_FACTOR);
        botRollerRPM = Math.abs(btmRoller.getSelectedSensorVelocity() * MOTOR_RPM_CONVERSION_FACTOR);

        // System.out.println("Target: " + topRollerTargetRPM);
        // System.out.println("Current: " + topRollerRPM);

        //System.out.println("TOp: " + topRollerRPM);
        //System.out.println("rotation per 100ms " + Math.abs(topRoller.getSelectedSensorVelocity()));
        //System.out.println("Btm: " + botRollerRPM);

        switch(shooterState)
        {
            case OFF:
                if(topRollerTargetRPM > 0.0)
                {
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

                if(shootingCounter >= SHOOTING_COUNTER_THRESHOLD)
                {
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
            topRoller.selectProfileSlot(TOP_PID_SLOT, 0);
            btmRoller.selectProfileSlot(TOP_PID_SLOT, 0);

            topRollerTargetRPM = SHOOT_VEL_HIGH_TOP;
            botRollerTargetRPM = SHOOT_VEL_HIGH_BOT;

            setTargetVelocity();
            //topRoller.set(ControlMode.PercentOutput, 0.32);
        }
        else if(midScore)
        {
            topRoller.selectProfileSlot(MID_PID_SLOT, 0);
            btmRoller.selectProfileSlot(MID_PID_SLOT, 0);

            topRollerTargetRPM = SHOOT_VEL_MID_TOP;
            botRollerTargetRPM = SHOOT_VEL_MID_BOT;

            setTargetVelocity();
            //topRoller.set(0.34);
        }
        else if(cubeTransfer)
        {
            topRoller.selectProfileSlot(CUBE_PID_SLOT, 0);
            btmRoller.selectProfileSlot(CUBE_PID_SLOT, 0);

            topRollerTargetRPM = SHOOT_VEL_CUBE_TRANSFER_TOP;
            botRollerTargetRPM = SHOOT_VEL_CUBE_TRANSFER_BOT;

            setTargetVelocity();
            //topRoller.set(0.36);
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

    // order of args: all scoring buttons, shoot, abort
    public void cmdProcShooterBetter(boolean... inputs)
    {
        int stateIndex = -1;
        for(int i=0; i<targetStatesTop.length; i++){
            if(inputs[i]){
                stateIndex = i;
                break;
            }
        }

        if (stateIndex == -1){
            if(inputs[inputs.length - 2]){
                shoot();
            }
            if(inputs[inputs.length - 1]){
                shooterOff();
            }
        }

        topRoller.selectProfileSlot(stateIndex, 0);
        btmRoller.selectProfileSlot(stateIndex, 0);
        
        topRollerTargetRPM = stateMapTop.get(targetStatesTop[stateIndex])[4];
        topRollerTargetRPM = stateMapBtm.get(targetStatesBtm[stateIndex])[4];
    }

    public void printTemperatures()
    {
        System.out.println("Top: " + topRoller.getTemperature());
        System.out.println("Btm: " + btmRoller.getTemperature());
    }

    private void shoot()
    {
        shooterState = ShooterState.SHOOTING;
        feeder.set(FEEDER_SPEED);
    }

    private void shooterOff()
    {
        shooterState = ShooterState.OFF;

        shootingCounter = 0;

        Robot.xboxAux.setRumble(RumbleType.kLeftRumble, 0.0);

        topRollerTargetRPM = 0.0;
        botRollerTargetRPM = 0.0;
        
        //turn pid off

        topRoller.set(ControlMode.PercentOutput, 0.0);
        btmRoller.set(ControlMode.PercentOutput, 0.0);
        feeder.set(0);
    }

    private void setTargetVelocity()
    {
        double velTop = -Conversions.RPMToFalcon(topRollerTargetRPM, 1.0);
        topRoller.set(ControlMode.Velocity, velTop);

        double velBot = Conversions.RPMToFalcon(botRollerTargetRPM, 1.0);
        btmRoller.set(ControlMode.Velocity, velBot);
        System.out.println("velTop value " + Math.abs(velTop));

        shooterState = ShooterState.WAIT_FOR_STEADY;
    }

    public void smartdashboardShooter()
    {
        // SmartDashboard.putNumber("top sensor velocity", topRoller.getSelectedSensorVelocity());
        // SmartDashboard.putNumber("btm sensor velocity", btmRoller.getSelectedSensorVelocity());
        SmartDashboard.putNumber("toproller RPM", topRollerRPM);
        SmartDashboard.putNumber("toproller rotation per 100ms", Math.abs(topRoller.getSelectedSensorVelocity()));
        SmartDashboard.putNumber("bottomroller RPM", botRollerRPM);
    }
}
