// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.WristSubsystem;

public class SetFinAngleCmd extends InstantCommand {

  WristSubsystem wrist;
  double angle;

  public SetFinAngleCmd(WristSubsystem wrist, double angle) {
    this.wrist = wrist;
    this.angle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wrist.setGoalAngle(angle);
  }
}
