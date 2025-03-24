// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ApproachSubsystem extends SubsystemBase {

  private CommandScheduler scheduler = CommandScheduler.getInstance();
  private CommandSwerveDrivetrain drivetrain;
  private PathPlannerPath path;
  
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
    SmartDashboard.putString("Tag Info", String.format("ID%d X:%.3f Y:%.3f T:%.1f", targetPos.tagId, targetPos.tagPose.getX(), targetPos.tagPose.getY(), targetPos.tagPose.getRotation().getDegrees()));

    // Create a list of three waypoints.
    // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
      drivetrain.getState().Pose,
      targetPos.pt1,
      targetPos.pt2
    );

    // limit severity of motion.
    PathConstraints constraints = new PathConstraints(2.0, 1.5, 2 * Math.PI, 4 * Math.PI); 
    
    // Create and return the path using the waypoints created above
    path = new PathPlannerPath(
        waypoints,
        constraints,
        null,
        targetPos.goalEndState); // Goal end state.

    path.preventFlipping = true;
    return AutoBuilder.followPath(path);
  }
}
