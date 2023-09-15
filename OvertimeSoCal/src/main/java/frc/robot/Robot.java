// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Mechanisms.CatzShooter;

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

   private final CatzShooter shooter = new CatzShooter();


   private final int XBOX_AUX_PORT = 1;
   private XboxController xboxAux;

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
   public void teleopInit()
   {

   }

   @Override
   public void teleopPeriodic()
   {
      shooter.cmdProcShooter(xboxAux.getRightTriggerAxis() > 0.1, xboxAux.getAButton());
   }
}
