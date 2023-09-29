package frc.Mechanisms.intake;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.Mechanisms.indexer.CatzIndexer;
import frc.Utils.CatzConstants;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

@SuppressWarnings("unused")
public class CatzIntake{

    private static CatzIntake instance = null;
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    //returns itself for singleton implementation
    public static CatzIntake getInstance()
    {
        if(instance == null)
        {
            instance = new CatzIntake();
        }

        return instance;
    }
    
    /*
     * 
     * [IMPORTANT]                                      [IMPORTANT]
     * [IMPORTANT]    ALL CONSTANTS ARE DUMMY VALUES    [IMPORTANT]
     * [IMPORTANT]                                      [IMPORTANT]
     *  
     */

    

    private final double INTAKE_ROLLER_POWER = 1.0;


    private final int INTAKE_GROUND    = 1;
    private final int INTAKE_STOWED   = 0;

    private int intakeState = INTAKE_STOWED;

    private CatzIntake() 
    {        
        switch(CatzConstants.currentMode)
        {
            case REAL:
                io = new IntakeReal();
                break;
            case SIM :
                io = null;
                break;
            default:
                io = new IntakeReal() {};
                break;
        }
    }

    public void intakePeriodic()
    {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Intake", inputs);
    }

    public void cmdProcIntake(boolean toggleDeployStow, double intake, double outtake)
    {
        if (toggleDeployStow && isStowed())
        {
            io.setIntakeSolenoidIO(Value.kForward);
            intakeState = INTAKE_GROUND;
        }
        else if (toggleDeployStow)
        {
            io.setIntakeSolenoidIO(Value.kReverse);
            intakeState = INTAKE_STOWED;
        }

        if (intake > 0.1)
        {
            io.setIntakeRollersPercentIO(-INTAKE_ROLLER_POWER);
            //Robot.indexer.indexerIntake(); TBD uncomment
        }
        else if (outtake > 0.1)
        {
            io.setIntakeRollersPercentIO(INTAKE_ROLLER_POWER);
            //Robot.indexer.indexerOuttake(); TBD uncomment
        }
    }


    public boolean isStowed() 
    {
        if (inputs.intakeSolenoidValue == Value.kForward) 
        {
            //intakeState = INTAKE_GROUND;
            return false;
        }
        else
        {
            //intakeState = INTAKE_STOWED;
            return true;
        }
    }
    // public void cmdProcIntake(boolean toggleIntake, double intakeRollerInward, double intakeRollerOutward)
    // {
    //     if (toggleIntake && isStowed())
    //     {
    //         intakeSolenoid.set(Value.kForward);
    //         intakeState = INTAKE_GROUND;
    //     }
    //     else if (toggleIntake)
    //     {
    //         intakeSolenoid.set(Value.kReverse);
    //         intakeState = INTAKE_STOWED;
    //     }

    //     if (intakeRollerInward > 0.1)
    //     {
    //         intakeRoller.set(ControlMode.PercentOutput, INTAKE_ROLLER_POWER);
    //     }
    //     else if (intakeRollerOutward > 0.1)
    //     {
    //         intakeRoller.set(ControlMode.PercentOutput, -INTAKE_ROLLER_POWER);
    //     }
    // }


}