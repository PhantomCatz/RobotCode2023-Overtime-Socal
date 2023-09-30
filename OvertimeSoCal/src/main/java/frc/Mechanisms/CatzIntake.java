package frc.Mechanisms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import frc.Mechanisms.CatzIndexer;

public class CatzIntake{
    /*
     * 
     * [IMPORTANT]                                      [IMPORTANT]
     * [IMPORTANT]    ALL CONSTANTS ARE DUMMY VALUES    [IMPORTANT]
     * [IMPORTANT]                                      [IMPORTANT]
     *  
     */

    private WPI_TalonFX intakeRoller;

   // CatzIndexer index = new CatzIndexer();

    private final int INTAKE_ROLLER_CAN_ID = 30;

    private final SupplyCurrentLimitConfiguration currentLimit;

    private final double  CURRENT_LIMIT_AMPS            = 60.0;
    private final double  CURRENT_LIMIT_TRIGGER_AMPS    = 60.0;
    private final double  CURRENT_LIMIT_TIMEOUT_SECONDS = 0.5;
    private final boolean ENABLE_CURRENT_LIMIT          = true;

    private final int GROUND_PCM_PORT    = 3;
    private final int STOWED_PCM_PORT    = 4;

    private final double INTAKE_ROLLER_POWER = 0.5;

    private final PneumaticsModuleType PCM_TYPE = PneumaticsModuleType.REVPH;
    private DoubleSolenoid intakeSolenoid;

    private final int INTAKE_GROUND    = 1;
    private final int INTAKE_STOWED   = 0;

    private int intakeState = INTAKE_STOWED;

    public boolean intakeActive = false;
    //public boolean outtakeActive = false;

    public CatzIntake() {        

        intakeRoller =new WPI_TalonFX(INTAKE_ROLLER_CAN_ID);

        currentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT, CURRENT_LIMIT_AMPS, CURRENT_LIMIT_TRIGGER_AMPS, CURRENT_LIMIT_TIMEOUT_SECONDS);

        intakeRoller.configFactoryDefault();
        intakeRoller.setNeutralMode(NeutralMode.Coast);
        intakeRoller.configSupplyCurrentLimit(currentLimit);

        intakeSolenoid = new DoubleSolenoid(PCM_TYPE, GROUND_PCM_PORT, STOWED_PCM_PORT);

    }

    public boolean isStowed() 
    {
        return true;/* 
        if (intakeSolenoid.get() == Value.kForward) 
        {
            //intakeState = INTAKE_GROUND;
            return true;
        }
        else
        {
            //intakeState = INTAKE_STOWED;
            return true;
        }*/
    }

    public void intakeRollerInward()
    {
        intakeRoller.set(ControlMode.PercentOutput, INTAKE_ROLLER_POWER);
        intakeActive = true;
    }

    public void intakeRollerOutward()
    {
        intakeRoller.set(ControlMode.PercentOutput, INTAKE_ROLLER_POWER);
        Robot.indexer.indexerOuttake(); 
    }

    public void deployIntake()
    {
        intakeSolenoid.set(Value.kForward);
    }

    public void stowIntake()
    {
        intakeSolenoid.set(Value.kReverse);
    }

    public void intakeRollerOff()
    {
        intakeRoller.set(0.0);

    }

    public void cmdProcIntake(boolean deployIntake, boolean stowIntake, double intake, double outtake)
    {
        intakeActive = false;
        // outtakeActive = false;
        if (deployIntake)
        {
            deployIntake();
            intakeState = INTAKE_GROUND;
        }
        
        if (stowIntake)
        {
            stowIntake();
            intakeState = INTAKE_STOWED;

        }

        if (intake > 0.1)
        {
            intakeRollerInward();
        }
        else if (outtake > 0.1)
        {
            intakeRollerOutward();
        }
        else 
        {
            intakeRollerOff();
        }
    }

    public boolean getIntakeStatus()
    {
        return intakeActive;
    }
    
}