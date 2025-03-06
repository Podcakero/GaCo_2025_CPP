// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.NoSuchElementException;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ApproachSubsystem extends SubsystemBase {
  /** Creates a new ApproachSubsystem. */
  public ApproachSubsystem() {}

  private Command  approachCommand;
  private CommandScheduler scheduler = CommandScheduler.getInstance();
  public AprilTagFieldLayout tags = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark); //CHS uses andymark, worlds uses welded
  public Optional<Alliance> alliance = DriverStation.getAlliance();
  ApproachTarget targetIdentifier = ApproachTarget.UNKNOWN;

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("Approach Target", targetIdentifier.toString());
  }

  public void startApproach() {
    scheduler.schedule(approachCommand);
  }


  // Direct call tag without an Enum
  /*private void createPathCmd(int id, boolean isLeft){
    // Distance in meters
    double spacing = 0.45;
    double offset = 0.2;
    if(isLeft){
      offset = -offset;
    }
    
    double coords[] = getTagCoords(id);
    double x = (coords[0] + Math.cos(coords[2])*spacing) + Math.cos(coords[2]+Math.PI/2)*offset;
    double y = (coords[1] + Math.sin(coords[2])*spacing) + Math.sin(coords[2]+Math.PI/2)*offset;
    System.out.println("[" + coords[0] + "/" + x + ", " + coords[1] + "/" + y + "]; Rot: " + Math.toDegrees(coords[2]));
    approachCommand = AutoBuilder.pathfindToPose(
      new Pose2d(x, y, new Rotation2d(coords[2] + Math.PI)),
      new PathConstraints( 4.0, 3.0,
      Math.PI, Math.PI * 2),
      0.0 );
  }*/

  /* Update the command stored in "approachCommand" to navigate to the specified position **/
  public void createPathCmd(ApproachTarget targetPos){
    targetIdentifier = targetPos;
    // Distance in meters
    double spacing = 0.45; // 1/2 length of robot
    double offset = 0.165;   // Offset from the center to the pole

    if(targetPos.position == ReefSidePosition.LEFT){
      offset = -offset;
    } else if(targetPos.position == ReefSidePosition.CENTER){
      offset = 0;
      spacing = 0.45;
    }
    
    double coords[] = getTagCoords(targetPos.tagId);
    double x = (coords[0] + Math.cos(coords[2])*spacing) + Math.cos(coords[2]+Math.PI/2)*offset;
    double y = (coords[1] + Math.sin(coords[2])*spacing) + Math.sin(coords[2]+Math.PI/2)*offset;

    //System.out.println("[" + coords[0] + "/" + x + ", " + coords[1] + "/" + y + "]; Rot: " + Math.toDegrees(coords[2]));

    approachCommand = AutoBuilder.pathfindToPose(
      new Pose2d(x, y, new Rotation2d(coords[2] + Math.PI)),
      new PathConstraints( 2.0, 2.0,
      Math.PI, Math.PI * 2),
      0.0 );
  }

  private double[] getTagCoords(int id){
    try{
      alliance = DriverStation.getAlliance();
      if(alliance.get().equals(Alliance.Red)){
        id = id - 11;
      }
    } catch(NoSuchElementException e){
    System.out.println("No alliance color assigned");
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


}
