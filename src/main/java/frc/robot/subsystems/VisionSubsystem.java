
package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class VisionSubsystem extends SubsystemBase{

    PhotonCamera lowerCamera;
    PhotonPoseEstimator poseEstimator;
    Optional<EstimatedRobotPose>  estimatedRobotPose;
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    Pose3d robotPose;

    static final Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); 
                                            //Cam mounted facing forward, half a meter forward of center, half a meter up from center.

    public VisionSubsystem(){
        lowerCamera = new PhotonCamera("PC_Camera Left");

        // Construct PhotonPoseEstimator
        poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
        poseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.AVERAGE_BEST_TARGETS);
    }

    public void periodic(){

        estimatedRobotPose = getEstimatedGlobalPose();
        if (estimatedRobotPose.isPresent()){
            EstimatedRobotPose estPose = estimatedRobotPose.get();   
            
            Pose3d pose   = estPose.estimatedPose;
            Pose2d robotPose = pose.toPose2d();

            SmartDashboard.putBoolean("pose Present", true);
            SmartDashboard.putString("Pose 3d", pose.toString());
            SmartDashboard.putString("Pose 2d", robotPose.toString());

            Robot.m_field.setRobotPose(robotPose);

        } else {
            SmartDashboard.putBoolean("pose Present", false);

        }
    }

    /**
     * The latest estimated robot pose on the field from vision data. This may be empty. This should
     * only be called once per loop.
     *
     * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
     * {@link getEstimationStdDevs}
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
     *     used for estimation.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var change : lowerCamera.getAllUnreadResults()) {
            visionEst = poseEstimator.update(change);
        }
        return visionEst;
    }
}