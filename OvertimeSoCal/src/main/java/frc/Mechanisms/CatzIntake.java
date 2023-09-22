package frc.Mechanisms;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class CatzIntake{
    /*
     * 
     * [IMPORTANT]                                      [IMPORTANT]
     * [IMPORTANT]    ALL CONSTANTS ARE DUMMY VALUES    [IMPORTANT]
     * [IMPORTANT]                                      [IMPORTANT]
     * 
     * 
     */

    private  WPI_TalonFX intake_roller;

    private final int INTAKE_ROLLER_CAN_ID = 2637;

    private final SupplyCurrentLimitConfiguration currentLimit;

    private final double  CURRENT_LIMIT_AMPS            = 60.0;
    private final double  CURRENT_LIMIT_TRIGGER_AMPS    = 60.0;
    private final double  CURRENT_LIMIT_TIMEOUT_SECONDS = 0.5;
    private final boolean ENABLE_CURRENT_LIMIT          = true;

    public CatzIntake() {

        intake_roller =new WPI_TalonFX(INTAKE_ROLLER_CAN_ID);

        currentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT, CURRENT_LIMIT_AMPS, CURRENT_LIMIT_TRIGGER_AMPS, CURRENT_LIMIT_TIMEOUT_SECONDS);

    }


}