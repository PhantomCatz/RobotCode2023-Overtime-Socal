package frc.Mechanisms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class CatzShooter {
    /*
     * 
     * [IMPORTANT]                                      [IMPORTANT]
     * [IMPORTANT]    ALL CONSTANTS ARE DUMMY VALUES    [IMPORTANT]
     * [IMPORTANT]                                      [IMPORTANT]
     * 
     * 
     */

    private final WPI_TalonFX frontRoller; 
    private final WPI_TalonFX backRoller; 

    private final double kP = 1.0;
    private final double kI = 0.0;
    private final double kD = 0.0;

    private final int FRONT_ROLLER_CAN_ID = 999;
    private final int BACK_ROLLER_CAN_ID = 999;

    private final SupplyCurrentLimitConfiguration currentLimit;

    private final double  CURRENT_LIMIT_AMPS            = 60.0;
    private final double  CURRENT_LIMIT_TRIGGER_AMPS    = 60.0;
    private final double  CURRENT_LIMIT_TIMEOUT_SECONDS = 0.5;
    private final boolean ENABLE_CURRENT_LIMIT          = true;

    private final double FRONT_ROLLER_POWER = 1.0;
    private final double BACK_ROLLER_POWER = 1.0;

    public CatzShooter()
    {
        frontRoller = new WPI_TalonFX(FRONT_ROLLER_CAN_ID);
        backRoller = new WPI_TalonFX(BACK_ROLLER_CAN_ID);

        currentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT, CURRENT_LIMIT_AMPS, CURRENT_LIMIT_TRIGGER_AMPS, CURRENT_LIMIT_TIMEOUT_SECONDS);

        frontRoller.configFactoryDefault();
        frontRoller.setNeutralMode(NeutralMode.Coast);
        frontRoller.configSupplyCurrentLimit(currentLimit);

        frontRoller.config_kP(0, kP);
        frontRoller.config_kI(0, kI);
        frontRoller.config_kD(0, kD);


        backRoller.configFactoryDefault();
        backRoller.setNeutralMode(NeutralMode.Coast);
        backRoller.configSupplyCurrentLimit(currentLimit);

        backRoller.config_kP(0, kP);
        backRoller.config_kI(0, kI);
        backRoller.config_kD(0, kD);
    }

    public void cmdProcShooter(boolean rightTrigger, boolean a)
    {
        if(rightTrigger)
        {
            backRoller.set(ControlMode.PercentOutput, BACK_ROLLER_POWER);
        }
        else
        {
            backRoller.set(ControlMode.PercentOutput, 0.0);
        }

        if(a)
        {
            frontRoller.set(ControlMode.PercentOutput, FRONT_ROLLER_POWER);
        }
    }
}
