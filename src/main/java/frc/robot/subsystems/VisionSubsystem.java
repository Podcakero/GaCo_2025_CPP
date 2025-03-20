
package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;

public class VisionSubsystem extends SubsystemBase{

    CommandSwerveDrivetrain drivetrain;
    PhotonCamera photonCamera;
    PhotonPoseEstimator poseEstimator;
    Optional<EstimatedRobotPose>  estimatedRobotPose;
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    Pose3d robotPose;
    boolean safetyOverride = false;
    String cameraName;
    Vector<N3> visionMeasurementStdDevs;
   
    /**
     * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
     * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    
    public VisionSubsystem(CommandSwerveDrivetrain  drivetrain, String cameraName, Transform3d robotToCam, Vector<N3> stdDev){
        this.drivetrain = drivetrain;
        this.cameraName = cameraName;
        this.visionMeasurementStdDevs = stdDev;

        photonCamera = new PhotonCamera(cameraName);

        // Construct PhotonPoseEstimator
        poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
        poseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
    }

    public void setSafetyOverride(boolean safetyOverride){
        this.safetyOverride = safetyOverride;
    }


    public void periodic(){
        estimatedRobotPose = getEstimatedGlobalPose();
        if (estimatedRobotPose.isPresent()){
            EstimatedRobotPose estPose = estimatedRobotPose.get();   
            
            double timestampSeconds = estPose.timestampSeconds;
            Pose2d robotPose = estPose.estimatedPose.toPose2d();

            // send this new vision position to drivetrain to adjust odometry if we are within 1 M of out last position
            Translation2d newPosition = robotPose.getTranslation();
            Translation2d oldPosition = drivetrain.getState().Pose.getTranslation();
            double displacement = oldPosition.getDistance(newPosition);

            // validate the position before usig it.
            if ((displacement <= 1.0) || (DriverStation.isDisabled()) || safetyOverride) {
                drivetrain.addVisionMeasurement(robotPose, timestampSeconds, visionMeasurementStdDevs);
            }

            // dont display if locked out
            SmartDashboard.putString(cameraName + " Pose 2d", robotPose.toString());
        }
        
        SmartDashboard.putBoolean(cameraName + " Safety Override", safetyOverride);
        
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
        for (var change : photonCamera.getAllUnreadResults()) {
            visionEst = poseEstimator.update(change);
        }
        return visionEst;
    }
}