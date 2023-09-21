package frc.Mechanisms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

public class CatzIndexer {

    private WPI_TalonFX indexerMotorLt;
    private WPI_TalonFX indexerMotorRt;

    private final int INDEXER_MOTOR_CAN_ID_LT = 2637;
    private final int INDEXER_MOTOR_CAN_ID_RT = 2637;

    private final SupplyCurrentLimitConfiguration currentLimit;

    private final double  CURRENT_LIMIT_AMPS            = 60.0;
    private final double  CURRENT_LIMIT_TRIGGER_AMPS    = 60.0;
    private final double  CURRENT_LIMIT_TIMEOUT_SECONDS = 0.5;
    private final boolean ENABLE_CURRENT_LIMIT          = true;

    private final double INDEXER_MOTOR_POWER_ON    =  0.5; 
    private final double INDEXER_MOTOR_POWER_OFF   =  0.0;
    private final double INDEXER_MOTOR_POWER_REV   = -0.5;

    private DigitalInput indexerBeamBreak;

    private final int INDEXER_BEAM_BREAK_DIO_PORT = 2637;

    public CatzIndexer() 
    {
        indexerMotorLt = new WPI_TalonFX(INDEXER_MOTOR_CAN_ID_LT);
        indexerMotorRt = new WPI_TalonFX(INDEXER_MOTOR_CAN_ID_RT);

        currentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT, CURRENT_LIMIT_AMPS, CURRENT_LIMIT_TRIGGER_AMPS, CURRENT_LIMIT_TIMEOUT_SECONDS);

        indexerMotorLt.configFactoryDefault();
        indexerMotorLt.setNeutralMode(NeutralMode.Coast);
        indexerMotorLt.configSupplyCurrentLimit(currentLimit);

        indexerMotorRt.configFactoryDefault();
        indexerMotorRt.setNeutralMode(NeutralMode.Coast);
        indexerMotorRt.configSupplyCurrentLimit(currentLimit);
        
        indexerMotorRt.setInverted(true);
        indexerMotorRt.follow(indexerMotorLt);

        indexerBeamBreak = new DigitalInput(INDEXER_BEAM_BREAK_DIO_PORT);
    }

    public boolean getBeamBreak()
    {
        return indexerBeamBreak.get();
    }

    public void indexerOn()
    {
        indexerMotorLt.set(INDEXER_MOTOR_POWER_ON);
    }
     
    public void indexerOff()
    {
        indexerMotorLt.set(INDEXER_MOTOR_POWER_OFF);
    }

    public void indexerReverse()
    {
        indexerMotorLt.set(INDEXER_MOTOR_POWER_REV);
    }

    public void indexerControl(double power) 
    {
        indexerMotorLt.set(ControlMode.PercentOutput, power);
    }
}
