// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.HomeElevatorCmd;
import frc.robot.commands.JustIntakeCmd;
import frc.robot.commands.TriggerEventCmd;
import frc.robot.commands.WaitForTowerStateCmd;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ApproachSubsystem;
import frc.robot.subsystems.ApproachTarget;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Globals;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.TowerEvent;
import frc.robot.subsystems.TowerState;
import frc.robot.subsystems.TowerSubsystem;
import frc.robot.Constants.DriverConstants;

public class RobotContainer {
    //public static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    //public static final double MaxAngularRate = RotationsPerSecond.of(1.5).in(RadiansPerSecond); // was 0.75

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.DrivetrainConstants.kMaxVelocityMPS * 0.08).withRotationalDeadband(Constants.DrivetrainConstants.kMaxAngularVelocityRPS * 0.08) // Add a 8% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    //private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.FieldCentricFacingAngle rotateTo = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(Constants.DrivetrainConstants.kMaxVelocityMPS * 0.08)
            .withRotationalDeadband(Constants.DrivetrainConstants.kMaxAngularVelocityRPS * 0.08)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withHeadingPID(Constants.DrivetrainConstants.kPHeading, Constants.DrivetrainConstants.kIHeading, Constants.DrivetrainConstants.kDHeading);

    private final Telemetry logger = new Telemetry(Constants.DrivetrainConstants.kMaxVelocityMPS);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandJoystick       copilot_1 = new CommandJoystick(1);
    private final CommandJoystick       copilot_2 = new CommandJoystick(2);

    static final Transform3d robotToLowerCam = new Transform3d(new Translation3d(0.26, 0.00, 0.20), 
                                                          new Rotation3d(0,0,0));
    static final Vector<N3> lowerCamStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(5));

    static final Transform3d robotToUpperCam = new Transform3d(new Translation3d(-0.05, 0.00, 1.017), 
                                                          new Rotation3d(0,Math.toRadians(2.1), Math.PI));
    static final Vector<N3> upperCamStdDevs = VecBuilder.fill(1.0, 1.0, Units.degreesToRadians(10));

    // Instanciate subsystems
    public final Globals globals = new Globals();
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final ElevatorSubsystem elevator = new ElevatorSubsystem();
    public final WristSubsystem wrist = new WristSubsystem();
    public final TowerSubsystem tower = new TowerSubsystem(elevator, wrist, joystick);
    public final VisionSubsystem lowerVision = new VisionSubsystem(drivetrain, "LowerTagCamera", robotToLowerCam, lowerCamStdDevs);
    public final ApproachSubsystem approach = new ApproachSubsystem(drivetrain);
    public final LEDSubsystem led = new LEDSubsystem(0);

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    /* Intake and Goto Commands */
    private final JustIntakeCmd intakeAndGotoL1 = new JustIntakeCmd(tower, TowerEvent.GOTO_L1);
    private final JustIntakeCmd intakeAndGotoL2 = new JustIntakeCmd(tower, TowerEvent.GOTO_L2);
    private final JustIntakeCmd intakeAndGotoL3 = new JustIntakeCmd(tower, TowerEvent.GOTO_L3);
    private final JustIntakeCmd intakeAndGotoL4 = new JustIntakeCmd(tower, TowerEvent.GOTO_L4);

    /* Event Trigger Commands */
    private final TriggerEventCmd intakeCoral = new TriggerEventCmd(tower, TowerEvent.INTAKE_CORAL);
    private final TriggerEventCmd score = new TriggerEventCmd(tower, TowerEvent.SCORE);
    private final TriggerEventCmd intakeLowAlgae = new TriggerEventCmd(tower, TowerEvent.INTAKE_LOW_ALGAE);
    private final TriggerEventCmd intakeHighAlgae = new TriggerEventCmd(tower, TowerEvent.INTAKE_HIGH_ALGAE);
    private final TriggerEventCmd gotoL1 = new TriggerEventCmd(tower, TowerEvent.GOTO_L1);
    private final TriggerEventCmd gotoL3 = new TriggerEventCmd(tower, TowerEvent.GOTO_L3);

    /* Waiting Commands */
    private final WaitForTowerStateCmd waitForAlgae = new WaitForTowerStateCmd(tower, TowerState.WAITING_FOR_ALGAE);
    private final WaitForTowerStateCmd waitForLowering = new WaitForTowerStateCmd(tower, TowerState.PAUSING_AFTER_SCORING_CORAL);
    private final WaitForTowerStateCmd waitForHome = new WaitForTowerStateCmd(tower, TowerState.HOME);

    /* Home Elevator Command */
    private final HomeElevatorCmd homeElevator = new HomeElevatorCmd(elevator, tower);

    /* Instant Commands */
    private final Command scoreInstant = tower.runOnce(() -> tower.triggerEvent(TowerEvent.SCORE));

    private final Command collectCoralLeftInstant = drivetrain.runOnce(() -> faceCoralStation(true).schedule());
    private final Command collectCoralRightInstant = drivetrain.runOnce(() -> faceCoralStation(false).schedule());

    private final Command intakeLowAlgaeInstant = drivetrain.runOnce(() -> tower.triggerEvent(TowerEvent.INTAKE_LOW_ALGAE));
    private final Command intakeHighAlgaeInstant = drivetrain.runOnce(() -> tower.triggerEvent(TowerEvent.INTAKE_HIGH_ALGAE));

    private final Command seedFieldCentricInstant = drivetrain.runOnce(() -> drivetrain.seedFieldCentric());
    private final Command stopDrivetrainInstant = drivetrain.runOnce(() -> drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.0).withVelocityY(0.0)));
    
    private final Command enableSafetyOverrideInstant = lowerVision.runOnce(() -> lowerVision.setSafetyOverride(true));
    private final Command enableDirectToAlgaeInstant = Commands.runOnce(() -> tower.enableGoToDirectAlgae());

    private final Command homeTowerInstant = tower.runOnce(() -> tower.homeTower());
    private final Command tiltTowerInstant = tower.runOnce(() -> tower.tiltForward());

    private final Command gotoL1Instant = tower.runOnce(() -> tower.triggerEvent(TowerEvent.GOTO_L1));
    private final Command gotoL2Instant = tower.runOnce(() -> tower.triggerEvent(TowerEvent.GOTO_L2));
    private final Command gotoL3Instant = tower.runOnce(() -> tower.triggerEvent(TowerEvent.GOTO_L3));
    private final Command gotoL4Instant = tower.runOnce(() -> tower.triggerEvent(TowerEvent.GOTO_L4));

    private final Command startApproachInstant = approach.runOnce(() -> approach.startApproach());
    private final Command reefAInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_A));
    private final Command reefBInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_B));
    private final Command reefABInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_AB));
    private final Command reefCInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_C));
    private final Command reefDInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_D));
    private final Command reefCDInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_CD));
    private final Command reefEInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_E));
    private final Command reefFInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_F));
    private final Command reefEFInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_EF));
    private final Command reefGInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_G));
    private final Command reefHInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_H));
    private final Command reefGHInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_GH));
    private final Command reefIInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_I));
    private final Command reefJInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_J));
    private final Command reefIJInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_IJ));
    private final Command reefKInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_K));
    private final Command reefLInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_L));
    private final Command reefKLInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.REEF_KL));
    private final Command leftCoralStationInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.LEFT_SOURCE)).andThen(approach.runOnce(() -> approach.startApproach())).andThen(() -> tower.triggerEvent(TowerEvent.INTAKE_CORAL));
    private final Command rightCoralStationInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.RIGHT_SOURCE)).andThen(approach.runOnce(() -> approach.startApproach())).andThen(() -> tower.triggerEvent(TowerEvent.INTAKE_CORAL));
    private final Command approachBargeInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.BARGE));
    private final Command approachProcessorInstant = tower.runOnce(() -> approach.identifyTarget(ApproachTarget.PROCESSOR));

    /* Robot Centric Movement Commands */
    private final Command robotCentricForward = drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.75).withVelocityY(0));
    private final Command robotCentricBackward = drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.75).withVelocityY(0));
    private final Command robotCentricLeft = drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.0).withVelocityY(0.25));
    private final Command robotCentricRight = drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.0).withVelocityY(-0.25));

    public RobotContainer() {
        
        // All named commands =========================
        NamedCommands.registerCommand("INTAKE_CORAL",              intakeCoral);
        NamedCommands.registerCommand("INTAKE_AND_GOTO_L1",        intakeAndGotoL1);
        NamedCommands.registerCommand("INTAKE_AND_GOTO_L2",        intakeAndGotoL2);
        NamedCommands.registerCommand("INTAKE_AND_GOTO_L3",        intakeAndGotoL3);
        NamedCommands.registerCommand("INTAKE_AND_GOTO_L4",        intakeAndGotoL4);
        NamedCommands.registerCommand("SCORE_CORAL",               score);
        NamedCommands.registerCommand("SCORE_ALGAE",               score); // same as coral
        NamedCommands.registerCommand("GET_ALGAE",                 intakeHighAlgae);
        NamedCommands.registerCommand("GO_TO_L1",                  gotoL1);
        NamedCommands.registerCommand("WAIT_FOR_ALGAE",            waitForAlgae);
        NamedCommands.registerCommand("WAIT_FOR_LOWERING",         waitForLowering);
        NamedCommands.registerCommand("WAIT_FOR_HOME",             waitForHome);

        NamedCommands.registerCommand("GO_DIRECTLY_TO_ALGAE",      enableDirectToAlgaeInstant);
    
        // All Path Planner event triggers  ===========
        new EventTrigger("GOTO_L1_ALGAE").onTrue(gotoL1);
        new EventTrigger("GOTO_L3_ALGAE").onTrue(gotoL3);
        new EventTrigger("INTAKE_LOW_ALGAE").onTrue(intakeLowAlgae);
        new EventTrigger("INTAKE_HIGH_ALGAE").onTrue(intakeHighAlgae);
 
        // Configure Auto Chooser  ===============================
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        // Send drive module data to dashboard
        SmartDashboard.putData("Swerve Drive", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
              builder.setSmartDashboardType("SwerveDrive");
          
              builder.addDoubleProperty("Front Left Angle", () -> drivetrain.getModule(0).getPosition(false).angle.getRadians(), null);
              builder.addDoubleProperty("Front Left Velocity", () -> drivetrain.getModule(0).getDriveMotor().getVelocity().getValueAsDouble(), null);
          
              builder.addDoubleProperty("Front Right Angle", () -> drivetrain.getModule(1).getPosition(false).angle.getRadians(), null);
              builder.addDoubleProperty("Front Right Velocity", () -> drivetrain.getModule(1).getDriveMotor().getVelocity().getValueAsDouble(), null);
          
              builder.addDoubleProperty("Back Left Angle", () -> drivetrain.getModule(2).getPosition(false).angle.getRadians(), null);
              builder.addDoubleProperty("Back Left Velocity", () -> drivetrain.getModule(2).getDriveMotor().getVelocity().getValueAsDouble(), null);
          
              builder.addDoubleProperty("Back Right Angle", () -> drivetrain.getModule(3).getPosition(false).angle.getRadians(), null);
              builder.addDoubleProperty("Back Right Velocity", () -> drivetrain.getModule(3).getDriveMotor().getVelocity().getValueAsDouble(), null);
          
              builder.addDoubleProperty("Robot Angle", () -> drivetrain.getRotation3d().getMeasureAngle().baseUnitMagnitude(), null);
            }
          });
  
        configureBindings();

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * Constants.DrivetrainConstants.kMaxVelocityMPS * Constants.DriverConstants.kMaxDriveSpeed * tower.getTowerSpeedSafetyFactor()) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * Constants.DrivetrainConstants.kMaxVelocityMPS * Constants.DriverConstants.kMaxDriveSpeed  * tower.getTowerSpeedSafetyFactor()) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * Constants.DrivetrainConstants.kMaxAngularVelocityRPS * Constants.DriverConstants.kMaxTurnSpeed * tower.getTowerSpeedSafetyFactor()) // Drive counterclockwise with negative X (left)
            )
        );

        // avoid the PathPlanner startup delay....
        FollowPathCommand.warmupCommand().schedule();
    }

    private void configureBindings() {

        // Driver Buttons
        joystick.rightTrigger(0.5).onTrue(scoreInstant);  // score coral or algae

        joystick.back().onTrue(seedFieldCentricInstant);  // reset field centric home

        joystick.start().onTrue(homeElevator);  //home the elevator
        joystick.rightStick().onTrue(tiltTowerInstant); // Tilt the elevator

        joystick.leftBumper().onTrue(collectCoralLeftInstant);  // collect coral left side
        joystick.rightBumper().onTrue(collectCoralRightInstant);// collect coral right side

        joystick.y().onTrue(intakeHighAlgaeInstant);
        joystick.a().onTrue(intakeLowAlgaeInstant);

        joystick.x().onTrue(approachBargeInstant);
        joystick.b().onTrue(approachProcessorInstant);

        // ==== Approach Buttons ================================

        joystick.leftTrigger(0.5).onTrue(startApproachInstant)
        .onFalse(stopDrivetrainInstant);

        // ==== NON Field Centric driving ================================

        joystick.pov(0).whileTrue(robotCentricForward);
        joystick.pov(180).whileTrue(robotCentricBackward);
        joystick.pov(90).whileTrue(robotCentricLeft);
        joystick.pov(270).whileTrue(robotCentricRight);
            
        // ====  CoPilot 1 Buttons  ======================================

        copilot_1.button(DriverConstants.reset).onTrue(enableSafetyOverrideInstant
//                                             .andThen(upperVision.runOnce(() -> upperVision.setSafetyOverride(true)))  //   only override low cam safety to reposition
                                               );

        copilot_1.button(DriverConstants.home).onTrue(homeTowerInstant);

        copilot_1.button(DriverConstants.l1).onTrue(gotoL1Instant);
        copilot_1.button(DriverConstants.l2).onTrue(gotoL2Instant);
        copilot_1.button(DriverConstants.l3).onTrue(gotoL3Instant);
        copilot_1.button(DriverConstants.l4).onTrue(gotoL4Instant);

        copilot_1.button(DriverConstants.pose_i).onTrue(reefIInstant);
        copilot_1.button(DriverConstants.pose_j).onTrue(reefJInstant);
        copilot_1.button(DriverConstants.pose_ija).onTrue(reefIJInstant);
        copilot_1.button(DriverConstants.pose_k).onTrue(reefKInstant);
        copilot_1.button(DriverConstants.pose_l).onTrue(reefLInstant);
        copilot_1.button(DriverConstants.pose_kla).onTrue(reefKLInstant);
        
        // ===  CoPilot 2 Buttons  ===========================================
        copilot_2.button(DriverConstants.pose_a).onTrue(reefAInstant);
        copilot_2.button(DriverConstants.pose_b).onTrue(reefBInstant);
        copilot_2.button(DriverConstants.pose_aba).onTrue(reefABInstant);
        copilot_2.button(DriverConstants.pose_c).onTrue(reefCInstant);
        copilot_2.button(DriverConstants.pose_d).onTrue(reefDInstant);
        copilot_2.button(DriverConstants.pose_cda).onTrue(reefCDInstant);
        copilot_2.button(DriverConstants.pose_e).onTrue(reefEInstant);
        copilot_2.button(DriverConstants.pose_f).onTrue(reefFInstant);
        copilot_2.button(DriverConstants.pose_efa).onTrue(reefEFInstant);
        copilot_2.button(DriverConstants.pose_g).onTrue(reefGInstant);
        copilot_2.button(DriverConstants.pose_h).onTrue(reefHInstant);
        copilot_2.button(DriverConstants.pose_gha).onTrue(reefGHInstant);
       
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }

    // ==============================================================================================
    // Approach Command code
    // ==============================================================================================
    private Rotation2d targetAngle;
    private double headingError;

    BooleanSupplier atTarget = (() -> {
        
        //if(targetAngle - (drivetrain.getState().Pose.getRotation().getDegrees()) > 180){
        //    headingError = (targetAngle - (drivetrain.getState().Pose.getRotation().getDegrees())) - 360;
        //} else if(targetAngle - (drivetrain.getState().Pose.getRotation().getDegrees()) < -180){
        //    headingError = (targetAngle - (drivetrain.getState().Pose.getRotation().getDegrees())) + 360;
        //} else{
        //    headingError = targetAngle - (drivetrain.getState().Pose.getRotation().getDegrees());
        //}

        headingError = Math.abs(targetAngle.getDegrees() - drivetrain.getState().Pose.getRotation().getDegrees());

        if(headingError < 1) {
            return true;
        } else{
            SmartDashboard.putNumber("Degrees Left to Turn", headingError);
            return false;
        }
    });

    public Command faceCoralStation(boolean isLeft){
        // Left coral station is tag 13, right is 12. Uses getTagId to convert Blue april tag IDs to Red
        targetAngle = Constants.kFieldLayout.getTagPose(ApproachTarget.getTagId((isLeft) ? 13 : 12)).get().getRotation().toRotation2d();

        // if(targetAngle - (drivetrain.getState().Pose.getRotation().getDegrees()) > 180){
        //     headingError = (targetAngle - (drivetrain.getState().Pose.getRotation().getDegrees())) + 360;
        // } else{
        //     headingError = targetAngle - (drivetrain.getState().Pose.getRotation().getDegrees());
        // }
        return drivetrain.applyRequest(() ->
                // drive.withVelocityX(-joystick.getLeftY() * Constants.DrivetrainConstants.kMaxVelocityMPS * Constants.ApproachConstants.maxApproachLinearVelocityPercent) // Drive forward with negative Y (forward)
                //     .withVelocityY(-joystick.getLeftX() * Constants.DrivetrainConstants.kMaxVelocityMPS * Constants.ApproachConstants.maxApproachLinearVelocityPercent) // Drive left with negative X (left)
                //     .withRotationalRate(clamp((headingError * Constants.DrivetrainConstants.kMaxAngularVelocityRPS * Constants.ApproachConstants.maxApproachAngularVelocityPercent), -Math.PI, Math.PI, Math.PI/4)) // Auto rotate to position
                rotateTo.withTargetDirection(targetAngle)
                    .withVelocityX(-joystick.getLeftY() * Constants.DrivetrainConstants.kMaxVelocityMPS * Constants.ApproachConstants.maxApproachLinearVelocityPercent)
                    .withVelocityY(-joystick.getLeftX() * Constants.DrivetrainConstants.kMaxVelocityMPS * Constants.ApproachConstants.maxApproachLinearVelocityPercent)
            ).until(atTarget).andThen(() -> tower.triggerEvent(TowerEvent.INTAKE_CORAL)); // Run until target angle is reached
      }

    // private double clamp(double value, double min, double max, double innerBound){
    //     if(value > max){
    //         value = max;
    //     } else if(value > 0 && value < innerBound){
    //         value = innerBound;
    //     } else if(value < min){
    //         value = min;
    //     } else if(value < 0 && value > -innerBound){
    //         value = -innerBound;
    //     }
    //     return value;
    // }
}
