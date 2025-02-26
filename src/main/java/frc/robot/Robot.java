// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  public static final Field2d m_field = new Field2d();

  private final RobotContainer m_robotContainer;

  private final Timer m_gcTimer = new Timer();


  public Robot() {
    m_robotContainer = new RobotContainer();
    m_gcTimer.start();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 

    // run the garbage collector every 5 seconds (New added by Phil)
    //if (m_gcTimer.advanceIfElapsed(5)) {
    // System.gc();
    //}
  }

  @Override
  public void disabledInit() {
    SignalLogger.enableAutoLogging(false);
    m_robotContainer.elevator.resetRelativeEncoder();
    SmartDashboard.putData("Field", m_field);
  }

  @Override
  public void disabledPeriodic() {
  
  }

  @Override
  public void disabledExit() {
    m_robotContainer.elevator.resetRelativeEncoder();
    m_robotContainer.elevator.resetElevatorControl();
    m_robotContainer.wrist.resetWristControl();
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    m_robotContainer.tower.homeTower();
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
