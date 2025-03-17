// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.NoSuchElementException;
import java.util.Optional;
import java.util.stream.Collectors;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ApproachSubsystem extends SubsystemBase {

  private Command approachCommand;
  private CommandScheduler scheduler = CommandScheduler.getInstance();
  public AprilTagFieldLayout tags = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark); //CHS uses andymark, worlds uses welded
  public Optional<Alliance> alliance = DriverStation.getAlliance();
  private CommandSwerveDrivetrain drivetrain;
  private PathPlannerPath path;
  
  public ApproachSubsystem(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("Approach Target", identifiedTarget.toString());
  }

  public void identifyTarget(ApproachTarget targetPos) {
    Globals.IDENTIFIED_TARGET = targetPos;
  }

  public void startApproach() {
    Globals.setLEDMode(LEDmode.APPROACH);
    if (Globals.IDENTIFIED_TARGET != ApproachTarget.UNKNOWN) {
      buildPathCmd(Globals.IDENTIFIED_TARGET);
      scheduler.schedule(approachCommand);
    }
  }

  /* Create a Path Command to navigate to the specified position **/
  public Command buildPathCmd(ApproachTarget targetPos){

    // Distance in meters
    double centerStandoff = 0.45;         // 1/2 length of robot
    double reefBranchOffset = 0.165;      // Offset from the center to the pole
    double closeApproachDistance = 0.15;  // Target Distance from reef when first approaching

    // Get tag coordinates and heading
    double tagX     = tags.getTagPose(getTagId(targetPos.tagId)).get().getX();
    double tagY     = tags.getTagPose(getTagId(targetPos.tagId)).get().getY();
    double tagAngle = tags.getTagPose(getTagId(targetPos.tagId)).get().getRotation().getAngle();
    double offsetX  = 0;
    double offsetY  = 0;    
    
    // adjust offset and standoff based on specific target location
    if(targetPos.position == ReefSidePosition.LEFT){
      reefBranchOffset = -reefBranchOffset;
    } else if(targetPos.position == ReefSidePosition.CENTER){
      reefBranchOffset = 0.0;
      centerStandoff += 0.45; // Space out further for algae
    }

    // Calculate left/right offsets for branch coordinates
    if (reefBranchOffset != 0) {
      offsetX  = Math.cos(tagAngle+Math.PI/2) * reefBranchOffset;
      offsetY  = Math.sin(tagAngle+Math.PI/2) * reefBranchOffset;
    }

    // Determine intermediate and final approach points
    double pt0X = drivetrain.getState().Pose.getX();
    double pt0Y = drivetrain.getState().Pose.getY();
    double pt1X = tagX + (Math.cos(tagAngle) * (centerStandoff + closeApproachDistance)) + offsetX;
    double pt1Y = tagY + (Math.sin(tagAngle) * (centerStandoff + closeApproachDistance)) + offsetY;
    double pt2X = tagX + (Math.cos(tagAngle) * (centerStandoff)) + offsetX;
    double pt2Y = tagY + (Math.sin(tagAngle) * (centerStandoff)) + offsetY;

    // determine trajectory angles for starting and ending path.
    Rotation2d initialAngle = Rotation2d.fromRadians(Math.atan2(pt1Y - pt0Y, pt1X - pt0X));
    Rotation2d finalAngle   = Rotation2d.fromRadians(reverseAngle(tagAngle));

    // Create a list of three waypoints.
    // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
      new Pose2d(pt0X, pt0Y, initialAngle),
      new Pose2d(pt1X, pt1Y, finalAngle),
      new Pose2d(pt2X, pt2Y, finalAngle)
    );

    // limit severity of motion.
    PathConstraints constraints = new PathConstraints(2.0, 1.5, 2 * Math.PI, 4 * Math.PI); 

    // Create and return the path using the waypoints created above
    path = new PathPlannerPath(
        waypoints,
        constraints,
        null,
        new GoalEndState(0.0, finalAngle) // Goal end state. 
    );

    path.preventFlipping = true;
    return AutoBuilder.followPath(path);
  }

  // Modify tag ID is running on Red Alliance.
  private int getTagId(int id){
    alliance = DriverStation.getAlliance();
    if(alliance.isPresent() && alliance.get().equals(Alliance.Red)){
      switch(id){
        case 12:
          id = 2;
          break;

        case 13:
          id = 1;
          break;

        case 14:
          id = 5;
          break;

        case 16:
          id = 3;
          break;

        case 17:
          id = 8;
          break;

        case 18:
          id = 7;
          break;

        case 19:
          id = 6;
          break;

        case 20:
          id = 11;
          break;

        case 21:
          id = 10;
          break;
          
        case 22:
          id = 9;
          break;
      }
    }

    return id;
  }

  /** Returns the reversed angle from the given angle in radians -PI to PI */
  
  private double reverseAngle(double angle){
    if(angle >= 0){  // This seems unnecessary  (Phil)
      return angle - Math.PI;
    } else{
      return angle + Math.PI;
    }
  }


}
