package frc.Mechanisms.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class ShooterIOReal implements ShooterIO 
{
    /*
     * 
     * [IMPORTANT]                                      [IMPORTANT]
     * [IMPORTANT]    ALL CONSTANTS ARE DUMMY VALUES    [IMPORTANT]
     * [IMPORTANT]                                      [IMPORTANT]
     * 
     * 
     */

    WPI_TalonFX topRoller;
    WPI_TalonFX btmRoller;


    private final double kF_TOP = 0.6; 
    private final double kP_TOP = 0.01; 
    private final double kI_TOP = 0.0;
    private final double kD_TOP = 0.0;

    private final double kF_BOT = 0.6; 
    private final double kP_BOT = 0.01; 
    private final double kI_BOT = 0.0;
    private final double kD_BOT = 0.0;


    private final int TOP_ROLLER_CAN_ID = 11; //see component map
    private final int BTM_ROLLER_CAN_ID  = 10;

    private final SupplyCurrentLimitConfiguration currentLimit;

    private final double  CURRENT_LIMIT_AMPS            = 60.0;
    private final double  CURRENT_LIMIT_TRIGGER_AMPS    = 60.0; //TBD Should be 55 current limited?
    private final double  CURRENT_LIMIT_TIMEOUT_SECONDS = 0.5;
    private final boolean ENABLE_CURRENT_LIMIT          = true;

    
    public ShooterIOReal()
    {
        topRoller = new WPI_TalonFX(TOP_ROLLER_CAN_ID);
        btmRoller = new WPI_TalonFX(BTM_ROLLER_CAN_ID);

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

    @Override
    public void updateInputs(ShooterIOInputs inputs)
    {
        inputs.topRollerSelectedSensorVelocity = topRoller.getSelectedSensorVelocity();
        inputs.btmRollerSelectedSensorVelocity = btmRoller.getSelectedSensorVelocity();
        inputs.topRollerTemp = topRoller.getTemperature();
        inputs.btmRollerTemp = btmRoller.getTemperature();
    }

    @Override
    public void setTopRollerVelocity(double velPwr)
    {
        topRoller.set(ControlMode.Velocity, velPwr);
    }

    @Override
    public void setBtmRollerVelocity(double velPwr)
    {
        btmRoller.set(ControlMode.Velocity, velPwr);
    }

    @Override
    public void setRollersOff()
    {
        topRoller.set(0.0);
        btmRoller.set(0.0);
    }


}
