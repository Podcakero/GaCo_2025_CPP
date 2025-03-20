// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.TowerSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HomeElevatorCmd extends Command {

  ElevatorSubsystem elevator;
  TowerSubsystem    tower;
  double motorCurrent = 0;
  int    overcurrentCount = 0;

  /** Creates a new DefaultWristCommand. */
  public HomeElevatorCmd(ElevatorSubsystem elevator, TowerSubsystem tower) {
    addRequirements(elevator);
    this.elevator = elevator;
    this.tower = tower;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.setSpeed(-0.15);
    overcurrentCount = 0;
  }

  @Override
  public void execute() {
    if (elevator.getCurrent() > 50) {
      overcurrentCount++;
    } else {
      overcurrentCount = 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.setSpeed(0);
    elevator.resetEncoder();
    elevator.resetElevatorControl();
    tower.initialize();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (overcurrentCount > 40);
  }
}
