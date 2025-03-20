// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Globals;
import frc.robot.subsystems.LEDmode;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  public static final Field2d m_field = new Field2d(); // Tele-Op field

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    SignalLogger.stop();
    CommandScheduler.getInstance().run(); 

  }

  @Override
  public void disabledInit() {
    SignalLogger.enableAutoLogging(false);
    SmartDashboard.putData("Field", m_field);
    SmartDashboard.putData("AutoField", Telemetry.m_field2);
    Globals.enableUpperCam();  // Use high cam in disabled
    m_robotContainer.tower.initialize();
  }

  @Override
  public void disabledPeriodic() {
    Telemetry.displayAutoPaths(); // Display the selected auto path on the dashboard while robot is disabled
  }

  @Override
  public void disabledExit() {
    m_robotContainer.tower.initialize();
    m_robotContainer.lowerVision.setSafetyOverride(false);
    /m_robotContainer.upperVision.setSafetyOverride(false);

  }

  @Override
  public void autonomousInit() {

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    m_robotContainer.tower.homeTower();
    Globals.disableUpperCam();  // ignore high cam in Auto
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.tower.initialize();
    Globals.setLEDMode(LEDmode.MANUAL );
    Globals.enableUpperCam();  // Use high cam in teleop
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
