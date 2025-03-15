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
  public ApproachTarget targetIdentifier = ApproachTarget.UNKNOWN;
  private PathPlannerPath path;

  public ApproachSubsystem(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
  }


  
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("Approach Target", targetIdentifier.toString());

  }

  public void startApproach() {
    Globals.setLEDMode(LEDmode.APPROACH);
    scheduler.schedule(approachCommand);
  }


   /* Update the command stored in "approachCommand" to navigate to the specified position **/
  public void createPathCmd(ApproachTarget targetPos){
    if(targetPos != ApproachTarget.UNKNOWN){
      targetIdentifier = targetPos;
      // Distance in meters
      double centerStandoff = 0.45; // 1/2 length of robot
      double reefBranchOffset = 0.165;   // Offset from the center to the pole
      double closeApproachDistance = 0.15;  // Distance to stay away from the reef when approaching close

      if(targetPos.position == ReefSidePosition.LEFT){
        reefBranchOffset = -reefBranchOffset;
      } else if(targetPos.position == ReefSidePosition.CENTER){
        reefBranchOffset = 0;
        centerStandoff += 0.45; // Space out further for algae
      }
      double tagCoords[] = getTagCoords(targetPos.tagId);  
      double tagAngle = tagCoords[2];

      double pt1X = tagCoords[0] + (Math.cos(tagAngle) * (centerStandoff + closeApproachDistance)) + Math.cos(tagAngle+Math.PI/2)*reefBranchOffset;
      double pt1Y = tagCoords[1] + (Math.sin(tagAngle) * (centerStandoff + closeApproachDistance)) + Math.sin(tagAngle+Math.PI/2)*reefBranchOffset;
      double pt2X = tagCoords[0] + (Math.cos(tagAngle) * (centerStandoff)) + Math.cos(tagAngle+Math.PI/2)*reefBranchOffset;
      double pt2Y = tagCoords[1] + (Math.sin(tagAngle) * (centerStandoff)) + Math.sin(tagAngle+Math.PI/2)*reefBranchOffset;

      Rotation2d initialAngle = Rotation2d.fromRadians(Math.atan2(pt1Y - drivetrain.getState().Pose.getY(), pt1X - drivetrain.getState().Pose.getX()));
      Rotation2d finalAngle = Rotation2d.fromRadians(reverseAngle(tagAngle));

      // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
      List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        new Pose2d(drivetrain.getState().Pose.getX(), drivetrain.getState().Pose.getY(), initialAngle),
        new Pose2d(pt1X, pt1Y, finalAngle),
        new Pose2d(pt2X, pt2Y, finalAngle)
      );

      PathConstraints constraints = new PathConstraints(2.0, 1.5, 2 * Math.PI, 4 * Math.PI); 

      // Create the path using the waypoints created above
      path = new PathPlannerPath(
          waypoints,
          constraints,
          null,
          new GoalEndState(0.0, finalAngle) // Goal end state. 
      );

      path.preventFlipping = true;
      approachCommand = AutoBuilder.followPath(path);
      
    } else {
      // No path is selected, do nothing
    }
  }

  private double[] getTagCoords(int id){
    alliance = DriverStation.getAlliance();
    if(alliance.isPresent() && alliance.get().equals(Alliance.Red)){
      switch(id){
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
    double coords[] = new double[3];
    coords[0] = Math.round((tags.getTagPose(id).get().getX()*10000));
    coords[0] = coords[0]/10000;
    coords[1] = Math.round((tags.getTagPose(id).get().getY()*10000));
    coords[1] = coords[1]/10000;
    coords[2] = (tags.getTagPose(id).get().getRotation().getAngle());
    //System.out.println("Tag " + id +": [" + coords[0] + ", " + coords[1] + "], Rotation: " + Math.round(Units.radiansToDegrees(coords[2])));
    return coords;
  }

  /** Returns the reversed angle from the given angle in radians -PI to PI */
  private double reverseAngle(double angle){
    if(angle >= 0){
      return angle - Math.PI;
    } else{
      return angle + Math.PI;
    }
  }


}
