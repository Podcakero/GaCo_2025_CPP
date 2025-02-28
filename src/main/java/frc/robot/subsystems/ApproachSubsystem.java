// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ApproachSubsystem extends SubsystemBase {
  /** Creates a new ApproachSubsystem. */
  public ApproachSubsystem() {}

  private Command  approachCommand;
  private CommandScheduler scheduler = CommandScheduler.getInstance();

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void startApproach() {
    scheduler.schedule(approachCommand);
  }

  public void setTarget(ApproachTarget target){
    switch (target) {
      case REEF_A: {
        approachCommand = AutoBuilder.pathfindToPose(
                            new Pose2d(3.179, 4.181, new Rotation2d(0)),
                            new PathConstraints( 1.0, 2.0,
                            Math.PI, Math.PI * 2),
                            0.0 );
        break;
      }

      case REEF_G: {
        approachCommand = AutoBuilder.pathfindToPose(
                            new Pose2d(5.795, 3.825, new Rotation2d(Math.PI)),
                            new PathConstraints( 1.0, 2.0,
                            Math.PI, Math.PI * 2),
                            0.0 );
        break;
      }

    }
  }
}
