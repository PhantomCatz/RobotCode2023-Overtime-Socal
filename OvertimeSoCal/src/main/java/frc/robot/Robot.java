// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.Mechanisms.CatzShooter;
import frc.Autonomous.CatzAutonomous;
import frc.Autonomous.CatzAutonomousPaths;
import frc.Autonomous.CatzBalance;
import frc.DataLogger.CatzLog;
import frc.DataLogger.DataCollection;
import frc.Mechanisms.CatzDrivetrain;
import frc.Mechanisms.CatzIndexer;
import frc.Mechanisms.CatzIntake;

public class Robot extends TimedRobot {

   public static DataCollection dataCollection;
   public ArrayList<CatzLog> dataArrayList;

   // ---------------------------------------------------------------------------------------------
   // Autonomous
   // ---------------------------------------------------------------------------------------------
   public static CatzAutonomous auton;
   public static CatzAutonomousPaths paths;
   public static CatzBalance balance;

   private final double OFFSET_DELAY = 0.5; // TBD put into AUTO BALANCE class

   // ---------------------------------------------------------------------------------------------
   // Mechanisms
   // ---------------------------------------------------------------------------------------------
   public static final CatzDrivetrain drivetrain = new CatzDrivetrain();
   public static final CatzIntake intake = new CatzIntake();
   public static final CatzIndexer indexer = new CatzIndexer();
   public static final CatzShooter shooter = new CatzShooter();

   public static final CatzConstants constants = new CatzConstants();

   public static XboxController xboxDrv;
   public static XboxController xboxAux;

   private final int XBOX_DRV_PORT = 0;
   private final int XBOX_AUX_PORT = 1;

   public static AHRS navX;
   public static Timer currentTime;

   @Override
   public void robotInit() {
      dataCollection = new DataCollection();
      dataArrayList = new ArrayList<CatzLog>();

      dataCollection.dataCollectionInit(dataArrayList);

      currentTime = new Timer();

      navX = new AHRS();
      navX.reset();

      xboxDrv = new XboxController(XBOX_DRV_PORT);
      xboxAux = new XboxController(XBOX_AUX_PORT);
      // shooter.printTemperatures();
   }

   @Override
   public void robotPeriodic() {
      shooter.smartdashboardShooter();
   }

   @Override
   public void autonomousInit() {
      drivetrain.setBrakeMode();
      currentTime.reset();
      currentTime.start();

      navX.reset();

      navX.setAngleAdjustment(-navX.getYaw() + 180.0); // set navx's zero position to opposite way robot is facing

      Timer.delay(OFFSET_DELAY); // TBD - This should be

      paths.executeSelectedPath();
   }

   @Override
   public void autonomousPeriodic() {
      shooter.shooterPeriodicUpdate();
   }

   @Override
   public void teleopInit() {
      currentTime.reset();
      currentTime.start();

      dataCollection.startDataCollection();
      balance.StopBalancing();
   }

   @Override
   public void teleopPeriodic() {
      drivetrain.cmdProcSwerve(0.0, xboxDrv.getLeftY() / 2, xboxDrv.getRightX(), navX.getAngle(),
            xboxDrv.getRightTriggerAxis());

      shooter.cmdProcShooter(xboxAux.getYButtonPressed(),
            xboxAux.getXButtonPressed(),
            xboxAux.getAButtonPressed(),
            xboxAux.getBButtonPressed(),
            xboxAux.getStartButtonPressed());

      shooter.shooterPeriodicUpdate();

      intake.cmdProcIntake(xboxAux.getRightStickButton(), xboxAux.getLeftTriggerAxis(), xboxAux.getRightTriggerAxis());
   }

   @Override
   public void disabledInit() {
      currentTime.stop();

      if (dataCollection.logDataValues == true) 
      {
         dataCollection.stopDataCollection();

         try {
            dataCollection.exportData(dataArrayList);
         } catch (Exception e) {
            e.printStackTrace();
         }
      }
   }
}