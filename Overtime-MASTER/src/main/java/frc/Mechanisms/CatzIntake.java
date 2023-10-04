package frc.Mechanisms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import frc.robot.Robot;

public class CatzIntake{

    private WPI_TalonFX intakeRoller;

    private final int INTAKE_ROLLER_CAN_ID = 30;

    private final double INTAKE_ROLLER_POWER = 0.45;
    private final double INTAKE_ROLLER_OFF   = 0.0;

    private final SupplyCurrentLimitConfiguration currentLimit;

    private final double  CURRENT_LIMIT_AMPS            = 60.0;
    private final double  CURRENT_LIMIT_TRIGGER_AMPS    = 60.0;
    private final double  CURRENT_LIMIT_TIMEOUT_SECONDS = 0.5;
    private final boolean ENABLE_CURRENT_LIMIT          = true;

    private final int DEPLOY_PCM_PORT    = 3;
    private final int STOWED_PCM_PORT    = 4;

    private final PneumaticsModuleType PCM_TYPE = PneumaticsModuleType.REVPH;
    private DoubleSolenoid intakeSolenoid;



    public CatzIntake() {    

        intakeRoller =new WPI_TalonFX(INTAKE_ROLLER_CAN_ID);

        currentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT, CURRENT_LIMIT_AMPS, CURRENT_LIMIT_TRIGGER_AMPS, CURRENT_LIMIT_TIMEOUT_SECONDS);

        intakeRoller.configFactoryDefault();
        intakeRoller.setNeutralMode(NeutralMode.Coast);
        intakeRoller.configSupplyCurrentLimit(currentLimit);

        intakeSolenoid = new DoubleSolenoid(PCM_TYPE, DEPLOY_PCM_PORT, STOWED_PCM_PORT);

        stowIntake();

    }



    public boolean isStowed() 
    {
        if (intakeSolenoid.get() == Value.kForward) 
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    public void deployIntake()
    {
        intakeSolenoid.set(Value.kReverse); //pneumatics is inverted
    }

    public void stowIntake()
    {
        intakeSolenoid.set(Value.kForward); //same issue
    }



    public void intakeRollerInward()
    {
        intakeRoller.set(ControlMode.PercentOutput, INTAKE_ROLLER_POWER); //0.5
        Robot.indexer.indexerIntake();
        Robot.shooter.intakeCube();
    }

    public void intakeRollerOutward()
    {
        intakeRoller.set(ControlMode.PercentOutput, -INTAKE_ROLLER_POWER); //0.5
        Robot.indexer.indexerOuttake(); 
    }

    public void intakeRollerOff()
    {
        intakeRoller.set(ControlMode.PercentOutput, INTAKE_ROLLER_OFF);
    }

    

    public void cmdProcIntake(boolean deployIntake, boolean stowIntake, double intake, double outtake)
    {
        
        if (stowIntake)
        {
            stowIntake();
        }
        else if (deployIntake)
        {
            deployIntake();
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
}