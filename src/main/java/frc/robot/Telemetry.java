package frc.robot;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.NoSuchElementException;
import java.util.stream.Collectors;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.RotationTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Telemetry {
    private final double MaxSpeed;

    /**
     * Construct a telemetry object, with the specified max speed of the robot
     * 
     * @param maxSpeed Maximum speed in meters per second
     */
    public Telemetry(double maxSpeed) {
        MaxSpeed = maxSpeed;
        //SignalLogger.start();
        SignalLogger.stop();
    }

    /* What to publish over networktables for telemetry */
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    /* Robot swerve drive state */
    private final NetworkTable driveStateTable = inst.getTable("DriveState");
    private final StructPublisher<Pose2d> drivePose = driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();
    private final StructPublisher<ChassisSpeeds> driveSpeeds = driveStateTable.getStructTopic("Speeds", ChassisSpeeds.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleStates = driveStateTable.getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleTargets = driveStateTable.getStructArrayTopic("ModuleTargets", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModulePosition> driveModulePositions = driveStateTable.getStructArrayTopic("ModulePositions", SwerveModulePosition.struct).publish();
    private final DoublePublisher driveTimestamp = driveStateTable.getDoubleTopic("Timestamp").publish();
    private final DoublePublisher driveOdometryFrequency = driveStateTable.getDoubleTopic("OdometryFrequency").publish();

    /* Robot pose for field positioning */
    private final NetworkTable table = inst.getTable("Pose");
    private final DoubleArrayPublisher fieldPub = table.getDoubleArrayTopic("Robot").publish();
    private final StringPublisher fieldTypePub = table.getStringTopic(".type").publish();

    /* Mechanisms to represent the swerve module states */
    private final Mechanism2d[] m_moduleMechanisms = new Mechanism2d[] {
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
    };
    /* A direction and length changing ligament for speed representation */
    private final MechanismLigament2d[] m_moduleSpeeds = new MechanismLigament2d[] {
        m_moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
    };
    /* A direction changing and length constant ligament for module direction */
    private final MechanismLigament2d[] m_moduleDirections = new MechanismLigament2d[] {
        m_moduleMechanisms[0].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[1].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[2].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[3].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
    };

    private final double[] m_poseArray = new double[3];
    private final double[] m_moduleStatesArray = new double[8];
    private final double[] m_moduleTargetsArray = new double[8];

    /** Accept the swerve drive state and telemeterize it to SmartDashboard and SignalLogger. */
    public void telemeterize(SwerveDriveState state) {
        /* Telemeterize the swerve drive state */
        drivePose.set(state.Pose);
        driveSpeeds.set(state.Speeds);
        driveModuleStates.set(state.ModuleStates);
        driveModuleTargets.set(state.ModuleTargets);
        driveModulePositions.set(state.ModulePositions);
        driveTimestamp.set(state.Timestamp);
        driveOdometryFrequency.set(1.0 / state.OdometryPeriod);

        /* Also write to log file */
        m_poseArray[0] = state.Pose.getX();
        m_poseArray[1] = state.Pose.getY();
        m_poseArray[2] = state.Pose.getRotation().getDegrees();
        for (int i = 0; i < 4; ++i) {
            m_moduleStatesArray[i*2 + 0] = state.ModuleStates[i].angle.getRadians();
            m_moduleStatesArray[i*2 + 1] = state.ModuleStates[i].speedMetersPerSecond;
            m_moduleTargetsArray[i*2 + 0] = state.ModuleTargets[i].angle.getRadians();
            m_moduleTargetsArray[i*2 + 1] = state.ModuleTargets[i].speedMetersPerSecond;
        }

        //SignalLogger.writeDoubleArray("DriveState/Pose", m_poseArray);
        //SignalLogger.writeDoubleArray("DriveState/ModuleStates", m_moduleStatesArray);
        //SignalLogger.writeDoubleArray("DriveState/ModuleTargets", m_moduleTargetsArray);
        //SignalLogger.writeDouble("DriveState/OdometryPeriod", state.OdometryPeriod, "seconds");

        /* Telemeterize the pose to a Field2d */
        fieldTypePub.set("Field2d");
        fieldPub.set(m_poseArray);

        /* Telemeterize the module states to a Mechanism2d */
        for (int i = 0; i < 4; ++i) {
            m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
            m_moduleDirections[i].setAngle(state.ModuleStates[i].angle);
            m_moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * MaxSpeed));

            SmartDashboard.putData("Module " + i, m_moduleMechanisms[i]);
        }
    }

    // Autonomous path display variables
    private static List<PathPlannerPath> paths;
    private static List<PathPlannerPath> pathsLast;
    private static List<Pose2d> poses;
    private static double autoAnimationStep = 0;
    private static final double AUTO_ANIMATION_SPEED = 0.25; // Speed of the animation. Default = 0.25 *Animation speed not to scale with the actual autonomous speed
    private static final int AUTO_ANIMATION_FREEZE_TIME = 10; // How long to wait before restarting the animation. Default = 10
    public static final Field2d m_field2 = new Field2d(); // Autonomous animation field
    
    /** Display the currently selected autonomous path on the dashboard */
    public static void displayAutoPaths(){
        //Retrieve the currently selected path from the dashboard
        try {
            paths = PathPlannerAuto.getPathGroupFromAutoFile(SmartDashboard.getEntry("Auto Mode/active").getString("None"));
        } catch (IOException e) {
            //System.out.println("Auto not found!"); // Selected auto program does not exist
            paths = null;
        } catch (ParseException e) {
            //System.out.println("Bad JSON in path file"); // The selected path cannot be parsed
        }

        if(paths != null && paths.size() > 0){
            // Flip paths to match current side of the field
            try{
                if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
                    for(int i = 0; i < paths.size(); i++){
                        paths.set(i, paths.get(i).flipPath());
                    }
                }
            } catch(NoSuchElementException e){
                // FMS is not connected, path will flip if needed once connected
            }
        }

        // Only runs this section once, when the selected auto changes
        if(paths != null && !paths.equals(pathsLast)){

            autoAnimationStep = -10;   // Give a slight delay after changing paths
            poses = new ArrayList<>(); // Clear the pose list
            m_field2.getObject("path").setPose(new Pose2d()); // Clear old paths

            // Generate trajectories for field2d and run animation
            if(paths != null && paths.size() > 0){

                // Add all Pose2d points from all paths in the selected auto to the poses variable
                for(int i = 0; i < paths.size(); i++){
                    poses.addAll(
                    paths.get(i).getAllPathPoints().stream()
                        .map(point -> new Pose2d(point.position, new Rotation2d()))
                        .collect(Collectors.toList()));
                }

                // Create a list of rotation values to find start and end rotations for each path
                List<RotationTarget> rotations = new ArrayList<>();
                for(int i = 0; i < paths.size(); i++){
                    rotations.addAll(
                    paths.get(i).getAllPathPoints().stream()
                        .map(point -> point.rotationTarget)
                        .collect(Collectors.toList()));
                }

                // Add rotations to the poses in each path by interpolating values between start and end rotation
                int startPoint = 0;     // The index of the starting point
                Double goalAngle = 0.0; // The angle the robot should be at after the path
                double toRotate = 0;    // The distance from the current angle to the goal angle

                for(int i = 0; i < poses.size(); i++){

                    if(rotations.get(i) != null){
                        RotationTarget tempRot = rotations.get(i); // Current rotational value (null unless at start or end of a path / at rotation checkpoint)

                        if(tempRot != null && tempRot.position() != 0){
                            
                            goalAngle = tempRot.rotation().getRadians();
                            
                            // Initialize heading in the first pose with ideal heading
                            if(startPoint == 0){
                                poses.set(startPoint, new Pose2d(poses.get(startPoint).getTranslation(), new Rotation2d((paths.get(0).getIdealStartingState().rotation().getRadians()))));
                                toRotate = goalAngle - poses.get(startPoint).getRotation().getRadians();
                                // Choose turning direction for shortest distance
                                if(toRotate > 3.1415){
                                    toRotate = (6.283 - (goalAngle - poses.get(startPoint).getRotation().getRadians()));
                                } else if(toRotate < -3.1415){
                                    toRotate = (6.283 + (goalAngle - poses.get(startPoint).getRotation().getRadians()));
                                }
                            } else{
                                toRotate = goalAngle - poses.get(startPoint-1).getRotation().getRadians();
                                // Choose turning direction for shortest distance
                                if(toRotate > 3.1415){
                                    toRotate = (6.283 - (goalAngle - poses.get(startPoint-1).getRotation().getRadians()));
                                } else if(toRotate < -3.1415){
                                    toRotate = (6.283 + (goalAngle - poses.get(startPoint-1).getRotation().getRadians()));
                                }
                                // Set rotation for first point in the path
                                poses.set(startPoint, new Pose2d(poses.get(startPoint).getTranslation(), new Rotation2d((toRotate/(i - (startPoint-1))) + poses.get(startPoint-1).getRotation().getRadians())));
                            }
                            // Increment rotation linearly throughout the path
                            for(int j = startPoint+1; j <= i; j++){
                                poses.set(j, new Pose2d(poses.get(j).getTranslation(), new Rotation2d((toRotate/(i - (startPoint-1))) + poses.get(j-1).getRotation().getRadians())));
                            }
                            startPoint = i+1; // Update the starting point to the next uninitialized one

                        }
                    }
                }
            }
            m_field2.getObject("Robot").setPose(poses.get(0)); // Reset the animation to the new starting pose
        }
        // Update the path, including with a blank one when no path is selected
        if(paths != pathsLast){
            m_field2.getObject("path").setPoses(poses); // Add the full auto to the field
            if(paths == null){
                m_field2.getObject("path").setPoses(new Pose2d()); // Overwrite with an empty path if paths is null
            }
        }

        if(poses != null){
            // Animate the auto by moving through the list of poses each cycle
            if(autoAnimationStep >= poses.size()-1){
                autoAnimationStep = -AUTO_ANIMATION_FREEZE_TIME; // Wait this many cycles before restarting the animation
            } else{
                autoAnimationStep = autoAnimationStep + AUTO_ANIMATION_SPEED; // Increment the animation counter
            }

            // If waiting period is over, set the pose of the robot to the current animation step
            if(autoAnimationStep >= 0 && paths != null){
                m_field2.getObject("Robot").setPose(poses.get((int)autoAnimationStep));
            } else if (paths == null){
                m_field2.getObject("Robot").setPose(new Pose2d());

            }
    
        }
        pathsLast = paths; // Store last path to see if path has been changed
    }

}
