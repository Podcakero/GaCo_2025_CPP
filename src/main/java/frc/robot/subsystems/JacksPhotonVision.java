
package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class JacksPhotonVision extends SubsystemBase{

    PhotonPipelineResult result;
    PhotonCamera cameraRight;
    PhotonCamera cameraLeft;
    List<PhotonTrackedTarget> targets = result.getTargets();
    PhotonTrackedTarget target = result.getBestTarget();
    int targetID = target.getFiducialId();
    Transform3d bestCameraToTarget = target.getBestCameraToTarget();
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    Transform3d cameraToRobot;
    Pose3d robotPose;

    double robotPoseX = robotPose.getX();
    double robotPoseY = robotPose.getY();
    double robotPoseZ = robotPose.getZ();


    public JacksPhotonVision(){
        cameraRight = new PhotonCamera("PC_Camera R");
        cameraLeft = new PhotonCamera("PC_Camera L");
    }

    

    public void periodic(){
        result = cameraRight.getLatestResult();
        SmartDashboard.putBoolean("cameraData" , result.hasTargets());
        
        if (aprilTagFieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
            robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), cameraToRobot);
        }   
        SmartDashboard.putNumber("robotPoseX", robotPoseX);
        SmartDashboard.putNumber("robotPoseY", robotPoseY);
        SmartDashboard.putNumber("robotPoseZ", robotPoseZ);
        
    }

    
}
