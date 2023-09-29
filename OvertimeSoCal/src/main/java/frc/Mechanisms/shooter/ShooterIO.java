package frc.Mechanisms.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO 
{
    @AutoLog
    public class ShooterIOInputs
    {
        public double topRollerSelectedSensorVelocity;
        public double btmRollerSelectedSensorVelocity;
        public double topRollerTemp;
        public double btmRollerTemp;

    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void setTopRollerVelocity(double velPwr) {}

    public default void setBtmRollerVelocity(double velPwr) {}

    public default void setRollersOff() {}


    
}
