package frc.Utils;

public class CatzConstants 
{
    public static final Mode currentMode = Mode.REAL;

    public static enum Mode {
      /** Running on a real robot. */
      REAL,
  
      /** Running a physics simulator. */
      SIM,
  
      /** Replaying from a log file. */
      REPLAY
    }
    
}
