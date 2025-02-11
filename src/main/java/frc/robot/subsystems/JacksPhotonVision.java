
package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class JacksPhotonVision extends SubsystemBase{

    PhotonPipelineResult result;
    PhotonCamera cameraRight;
    PhotonCamera cameraLeft;
    List<PhotonTrackedTarget> targets;
    PhotonTrackedTarget target;
    int targetID;
    Transform3d bestCameraToTarget;
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    Pose3d robotPose;

    double robotPoseX;
    double robotPoseY;
    double robotPoseZ;


    static final Transform3d cameraToRobot = new Transform3d(
        new Pose3d(0, 0, 0, new Rotation3d()),
            
        new Pose3d(0.0, 0.0, 0.0, new Rotation3d())
    );
    


    public JacksPhotonVision(){
        cameraRight = new PhotonCamera("PC_Camera Right");
        cameraLeft = new PhotonCamera("PC_Camera Left");
    }

    

    public void periodic(){

        result = cameraLeft.getLatestResult();

        SmartDashboard.putBoolean("cameraData" , result.hasTargets());
        
        if(result.hasTargets()){
            targets = result.getTargets();
            target = result.getBestTarget();
            targetID = target.getFiducialId();

            bestCameraToTarget = target.getBestCameraToTarget();

            

            if (aprilTagFieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
                
                robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),
                aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), 
                cameraToRobot
                );
                
                robotPoseX = robotPose.getX();
                robotPoseY = robotPose.getY();
                robotPoseZ = robotPose.getZ();

                Robot.m_field.setRobotPose(robotPose.toPose2d());

                SmartDashboard.putNumber("robotPoseX", robotPoseX);
                SmartDashboard.putNumber("robotPoseY", robotPoseY);
                SmartDashboard.putNumber("robotPoseZ", robotPoseZ);
            }   


            
        }
        
        
        
    }

    
}
