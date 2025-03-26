// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ApproachSubsystem extends SubsystemBase {

  private final CommandScheduler scheduler = CommandScheduler.getInstance();
  private final CommandSwerveDrivetrain drivetrain;
  private final PathConstraints pathConstraints = new PathConstraints(Constants.ApproachConstants.maxApproachLinearVelocityMPS, Constants.ApproachConstants.maxApproachLinearAccelerationMPSPS, Constants.ApproachConstants.maxApproachAngularVelocityRPS, Constants.ApproachConstants.maxApproachAngularAccelerationRPSPS);
  private PathPlannerPath path;
  private Pose2d pt0;
  private double pt0X;
  private double pt0Y;

  public ApproachSubsystem(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void identifyTarget(ApproachTarget targetPos) {
    Globals.IDENTIFIED_TARGET = targetPos;
  }

  public void startApproach() {
    Globals.setLEDMode(LEDmode.APPROACH);
    if (Globals.IDENTIFIED_TARGET != ApproachTarget.UNKNOWN) {
      scheduler.schedule(buildPathCmd(Globals.IDENTIFIED_TARGET));
    }
  }
  
  /* Create a Path Command to navigate to the specified position **/
  public Command buildPathCmd(ApproachTarget targetPos){
    
    // Create a list of three waypoints.
    // The rotation component of the pose is the direction of travel. 
    // Start moving towards pt1
    // End moving normal to tag surface
    pt0X = drivetrain.getState().Pose.getX();
    pt0Y = drivetrain.getState().Pose.getY();
    pt0 = new Pose2d(pt0X, pt0Y, new Rotation2d(targetPos.pt1.getX()-pt0X, targetPos.pt1.getY()-pt0Y) );
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(pt0, targetPos.pt1, targetPos.pt2);
    
    // Create and return the path using the waypoints created above
    path = new PathPlannerPath(
        waypoints,
        pathConstraints,
        null,
        targetPos.goalEndState); // Goal end state.
    path.preventFlipping = true;
    return AutoBuilder.followPath(path);
  }
}
