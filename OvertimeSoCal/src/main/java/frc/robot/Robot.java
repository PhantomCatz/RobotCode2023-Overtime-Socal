// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.Mechanisms.CatzShooter;
import frc.Mechanisms.CatzIndexer;
import frc.Mechanisms.CatzIntake;

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

   public static final CatzIntake intake = new CatzIntake();
   public static final CatzIndexer indexer = new CatzIndexer();
   public static final CatzShooter shooter = new CatzShooter();

   private final int XBOX_AUX_PORT = 1;
   public static XboxController xboxAux;


   @Override
   public void robotInit()
   {
      xboxAux = new XboxController(XBOX_AUX_PORT);
   }

   @Override
   public void robotPeriodic()
   {

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
      shooter.cmdProcShooter(xboxAux.getYButtonPressed(), xboxAux.getBButtonPressed(), xboxAux.getXButtonPressed(), xboxAux.getAButtonPressed());
      shooter.shooterPeriodicUpdate();

      intake.cmdProcIntake(xboxAux.getRightStickButton(), xboxAux.getLeftTriggerAxis(), xboxAux.getRightTriggerAxis());

   }
}