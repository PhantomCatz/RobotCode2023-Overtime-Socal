// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PneumaticHub;

import frc.Mechanisms.CatzShooter;
//import frc.Mechanisms.CatzIndexer;
import frc.Mechanisms.CatzIntake;
import frc.Mechanisms.CatzIndexer;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /*
   *  DUMMY CONTROLS:
   *    - Right Trigger: preroll
   *    - A: shoot 
   */

   public static CatzIntake  intake;
   public static CatzShooter shooter;
   public static CatzIndexer indexer;

   private final int XBOX_AUX_PORT = 1;
   public static XboxController xboxAux;

   private final int PH_CAN_ID = 1;

   public PneumaticHub pneumaticHub;

   @Override
   public void robotInit()
   {
      intake = new CatzIntake();
      indexer = new CatzIndexer();
      shooter = new CatzShooter();
      xboxAux = new XboxController(XBOX_AUX_PORT);
      pneumaticHub = new PneumaticHub(PH_CAN_ID);

      // shooter.printTemperatures();

   }

   @Override
   public void robotPeriodic()
   {
      shooter.smartdashboardShooter();
      System.out.println(indexer.getBeamBreak());
   }

   @Override
   public void autonomousPeriodic()
   {
      shooter.shooterPeriodicUpdate();
   }

   @Override
   public void teleopInit()
   {

   }

   @Override
   public void teleopPeriodic()
   {
      // shooter.cmdProcShooter(xboxAux.getYButtonPressed(), 
      //                        xboxAux.getXButtonPressed(), 
      //                        xboxAux.getAButtonPressed(), 
      //                        xboxAux.getBButtonPressed(),
      //                        xboxAux.getStartButtonPressed());
      
      shooter.shooterPeriodicUpdate();

      intake.cmdProcIntake(xboxAux.getRightStickButton(), xboxAux.getLeftStickButton(), xboxAux.getLeftTriggerAxis(), xboxAux.getRightTriggerAxis());
      indexer.cmdProcIndex(xboxAux.getLeftBumper(), xboxAux.getRightBumper(), xboxAux.getStartButton());
   }
}