// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PowerDistribution;

import frc.DataLogger.CatzLog;
import frc.DataLogger.DataCollection;

import frc.Mechanisms.CatzDrivetrain;
import frc.Mechanisms.CatzIntake;
import frc.Mechanisms.CatzIndexer;
import frc.Mechanisms.CatzShooter;

import frc.Autonomous.*;




/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot 
{
  //---------------------------------------------------------------------------------------------
  //  Shared Libraries & Utilities
  //---------------------------------------------------------------------------------------------
  public static CatzConstants       constants;

  public static DataCollection      dataCollection;
  public ArrayList<CatzLog>         dataArrayList;

  //----------------------------------------------------------------------------------------------
  //  Shared Robot Components (e.g. not mechanism specific, such as PDH, NavX, etc)
  //----------------------------------------------------------------------------------------------
  public static PowerDistribution PDH;

  public static AHRS              navX;

  public static Timer             currentTime;

  public static XboxController xboxDrv;
  public static XboxController xboxAux;

  private final int XBOX_DRV_PORT = 0;
  private final int XBOX_AUX_PORT = 1;

  public static final int DPAD_UP = 0;
  public static final int DPAD_DN = 180;
  public static final int DPAD_LT = 270;
  public static final int DPAD_RT = 90;

  public double  xboxGamePieceSelection = 0.0;
  public double  xboxElevatorManualPwr    = 0.1;
  public boolean xboxElevatorManualMode = false;
  public boolean xboxStowPos   = false;
  public boolean xboxLowNode   = false;
  public boolean xboxMidNode   = false;
  public boolean xboxHighNode  = false;
  public boolean xboxPickUpGroundPos = false;
  public boolean xboxPickUpSinglePos = false;
  public boolean xboxPickUpDoublePos = false;

  public static boolean xboxNULL      = false;

    
  
  //---------------------------------------------------------------------------------------------
  //  Autonomous
  //---------------------------------------------------------------------------------------------
  public static CatzAutonomous      auton;
  public static CatzAutonomousPaths paths;
  public static CatzBalance         balance;

  private final double OFFSET_DELAY = 0.5;    //TBD put into AUTO BALANCE class

  //---------------------------------------------------------------------------------------------
  //  Mechanisms
  //---------------------------------------------------------------------------------------------
  public static CatzDrivetrain   drivetrain;
  public static CatzShooter      shooter;
  public static CatzIndexer      indexer;
  public static CatzIntake       intake;


  /*-----------------------------------------------------------------------------------------
  *  
  *  robotXxx
  *
  *----------------------------------------------------------------------------------------*/
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() 
  {
    //-----------------------------------------------------------------------------------------
    //  Shared Libraries & Utilities
    //-----------------------------------------------------------------------------------------
    constants      = new CatzConstants();

    dataCollection = new DataCollection();
    dataArrayList  = new ArrayList<CatzLog>();
    
    dataCollection.dataCollectionInit(dataArrayList);


    //-----------------------------------------------------------------------------------------
    //  Shared Robot Components (e.g. not mechanism specific)
    //-----------------------------------------------------------------------------------------
    PDH = new PowerDistribution();

    navX = new AHRS();
    navX.reset();

    xboxDrv = new XboxController(XBOX_DRV_PORT);
    xboxAux = new XboxController(XBOX_AUX_PORT);

    currentTime = new Timer();

    //----------------------------------------------------------------------------------------------
    //  Autonomous
    //----------------------------------------------------------------------------------------------
    auton   = new CatzAutonomous();
    paths   = new CatzAutonomousPaths();
    balance = new CatzBalance();

    //----------------------------------------------------------------------------------------------
    //  Mechanisms
    //----------------------------------------------------------------------------------------------
    drivetrain = new CatzDrivetrain();
    intake     = new CatzIntake();
    indexer    = new CatzIndexer();
    shooter    = new CatzShooter();


    //led        = new CatzRGB();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic()
  {
    //----------------------------------------------------------------------------------------------
    //  Update status, LED's
    //----------------------------------------------------------------------------------------------
    
    dataCollection.updateLogDataID(); //NEEDS TO BE FIXED..causes robot to crash

    //led.LEDWork();

    //----------------------------------------------------------------------------------------------
    //  Shuffleboard Data Display
    //----------------------------------------------------------------------------------------------
    SmartDashboard.putNumber("NavX", navX.getAngle());


    drivetrain.smartDashboardDriveTrain();
       balance.SmartDashboardBalance();
       shooter.smartdashboardShooter();
       indexer.smartdashboardIndexer();
    

  
    //debug should be commented out for comp
    drivetrain.smartDashboardDriveTrain_DEBUG();
       balance.SmartDashboardBalanceDebug();
       
       
  }



  /*-----------------------------------------------------------------------------------------
  *  
  *  autonomousXxx
  *
  *----------------------------------------------------------------------------------------*/
  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() 
  {
    drivetrain.setBrakeMode();
    currentTime.reset();
    currentTime.start();

    navX.reset();

    navX.setAngleAdjustment(navX.getYaw()); //set navx's zero position to opposite way robot is facing
    
    Timer.delay(OFFSET_DELAY);  //TBD

    paths.executeSelectedPath();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic()
  {

  }



  /*-----------------------------------------------------------------------------------------
  *  
  *  teleopXxx
  *
  *----------------------------------------------------------------------------------------*/
  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit()
  {

    currentTime.reset();
    currentTime.start();

    dataCollection.startDataCollection();
    balance.StopBalancing();
    // drivetrain.printOffsetAverages();
  }


  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic()
  {
  
     drivetrain.cmdProcSwerve(xboxDrv.getLeftX(), xboxDrv.getLeftY(), xboxDrv.getRightX(), navX.getAngle(), xboxDrv.getRightTriggerAxis());
 
    // drivetrain.driveStraight(0.3); //for testing pruposes

    if(xboxDrv.getStartButtonPressed())
    {
      drivetrain.zeroGyro();
    }
    

    intake.cmdProcIntake(xboxDrv.getRightStickButton(),      //Right Stick Button DRV:  Deploy 
                         xboxDrv.getLeftStickButton(),       //Left Stick Button DRV:   Stow
                         xboxDrv.getRightTriggerAxis(),     //Right Trigger Axis DRV:   Intake
                         xboxDrv.getLeftTriggerAxis());     //Left  Trigger Axis DRV:  Outtake 
 
    indexer.cmdProcIndexer(xboxAux.getRightBumper(),         //Right Bumper Aux:       indexer inward
                           xboxAux.getLeftBumper(),          //Left Bumper Aux:        indexer outward
                           xboxAux.getLeftStickButton());    //Left Stick Aux:         indexer off 

    shooter.cmdProcShooter(xboxAux.getYButtonPressed(),      //Y-button Aux:           highscore  
                           xboxAux.getXButtonPressed(),      //X-button Aux:           midscore
                           xboxAux.getAButtonPressed(),      //A-button Aux:           cubetransfer
                           xboxAux.getBButtonPressed(),      //B-button Aux:           launch
                           xboxAux.getStartButtonPressed()); //Start-Button Aux:       shooter off
                  
    
  }



  /*-----------------------------------------------------------------------------------------
  *  
  *  disabledXxx
  *
  *----------------------------------------------------------------------------------------*/
  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit()
  {
    currentTime.stop();
    
    if(dataCollection.logDataValues == true)
    {
      dataCollection.stopDataCollection();

      try 
      {
        dataCollection.exportData(dataArrayList);
      } 
      catch (Exception e) 
      {
        e.printStackTrace();
      }
    }
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic()
  {

  }


  /*-----------------------------------------------------------------------------------------
  *  
  *  testXxx
  *
  *----------------------------------------------------------------------------------------*/
  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}


  /*-----------------------------------------------------------------------------------------
  *  
  *  simulateXxx
  *
  *----------------------------------------------------------------------------------------*/
  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}


  /*-----------------------------------------------------------------------------------------
  *  
  *  Misc
  *
  *----------------------------------------------------------------------------------------*/  
  public void zeroGyro()
  {
    navX.setAngleAdjustment(-navX.getYaw());
  }

}