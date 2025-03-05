// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.BooleanSupplier;

import org.ejml.equation.IntegerSequence.Range;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.TowerEvent;
import frc.robot.subsystems.TowerState;
import frc.robot.subsystems.TowerSubsystem;
import frc.robot.Constants.DriverConstants;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandJoystick       copilot_1 = new CommandJoystick(1);
    private final CommandJoystick       copilot_2 = new CommandJoystick(2);

    // Instanciate subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final ElevatorSubsystem elevator = new ElevatorSubsystem();
    public final WristSubsystem wrist = new WristSubsystem();
    public final TowerSubsystem tower = new TowerSubsystem(elevator, wrist);
    public final VisionSubsystem vision = new VisionSubsystem(drivetrain);
    public final ApproachSubsystem approach = new ApproachSubsystem();
    
    public final LEDSubsystem led = new LEDSubsystem(0);

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        
        NamedCommands.registerCommand("INTAKE_CORAL",   new TriggerEventCmd(tower, TowerEvent.INTAKE_CORAL));
        NamedCommands.registerCommand("INTAKE_AND_GOTO_L1",        new JustIntakeCmd(tower, TowerEvent.GOTO_L1));
        NamedCommands.registerCommand("INTAKE_AND_GOTO_L2",        new JustIntakeCmd(tower, TowerEvent.GOTO_L2));
        NamedCommands.registerCommand("INTAKE_AND_GOTO_L3",        new JustIntakeCmd(tower, TowerEvent.GOTO_L3));
        NamedCommands.registerCommand("INTAKE_AND_GOTO_L4",        new JustIntakeCmd(tower, TowerEvent.GOTO_L4));
        NamedCommands.registerCommand("SCORE_CORAL",    new TriggerEventCmd(tower, TowerEvent.SCORE_CORAL));

        NamedCommands.registerCommand("WAIT_FOR_LOWERING",         new WaitForTowerStateCmd(tower, TowerState.LOWERING));
        NamedCommands.registerCommand("WAIT_FOR_HOME",             new WaitForTowerStateCmd(tower, TowerState.HOME));

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

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
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * Constants.DriverConstants.kMaxDriveSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed * Constants.DriverConstants.kMaxDriveSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate * Constants.DriverConstants.kMaxTurnSpeed) // Drive counterclockwise with negative X (left)
            )
        );
    }

    private void configureBindings() {

        // Driver Buttons
        // Tower State Machine Events
        joystick.leftBumper().onTrue(tower.runOnce(() -> tower.triggerEvent(TowerEvent.INTAKE_CORAL)));
        joystick.rightTrigger(0.5).onTrue(tower.runOnce(() -> tower.triggerEvent(TowerEvent.SCORE_CORAL)));

        // ==== Approach Buttons ================================
        joystick.leftTrigger(0.5).onTrue(approach.runOnce(() -> approach.startApproach()))
        .onFalse(drivetrain.runOnce(() -> drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.0).withVelocityY(0.0))));

        // CoPilot 1 Buttons

        copilot_1.button(DriverConstants.reset).onTrue(tower.runOnce(() -> tower.homeTower()));

        copilot_1.button(DriverConstants.l1).onTrue(tower.runOnce(() -> tower.triggerEvent(TowerEvent.GOTO_L1)));
        copilot_1.button(DriverConstants.l2).onTrue(tower.runOnce(() -> tower.triggerEvent(TowerEvent.GOTO_L2)));
        copilot_1.button(DriverConstants.l3).onTrue(tower.runOnce(() -> tower.triggerEvent(TowerEvent.GOTO_L3)));
        copilot_1.button(DriverConstants.l4).onTrue(tower.runOnce(() -> tower.triggerEvent(TowerEvent.GOTO_L4)));

        copilot_1.button(DriverConstants.pose_i).onTrue(tower.runOnce(() -> approach.createPathCmd(ApproachTarget.REEF_I)));
        copilot_1.button(DriverConstants.pose_j).onTrue(tower.runOnce(() -> approach.createPathCmd(ApproachTarget.REEF_J)));
        copilot_1.button(DriverConstants.pose_k).onTrue(tower.runOnce(() -> approach.createPathCmd(ApproachTarget.REEF_K)));
        copilot_1.button(DriverConstants.pose_l).onTrue(tower.runOnce(() -> approach.createPathCmd(ApproachTarget.REEF_L)));
        
        // CoPilot 2 Buttons

        copilot_2.button(DriverConstants.reset).onTrue(tower.runOnce(() -> tower.triggerEvent(TowerEvent.HOME_TOWER)));
        copilot_2.button(DriverConstants.pose_a).onTrue(tower.runOnce(() -> approach.createPathCmd(ApproachTarget.REEF_A)));
        copilot_2.button(DriverConstants.pose_b).onTrue(tower.runOnce(() -> approach.createPathCmd(ApproachTarget.REEF_B)));
        copilot_2.button(DriverConstants.pose_c).onTrue(tower.runOnce(() -> approach.createPathCmd(ApproachTarget.REEF_C)));
        copilot_2.button(DriverConstants.pose_d).onTrue(tower.runOnce(() -> approach.createPathCmd(ApproachTarget.REEF_D)));
        copilot_2.button(DriverConstants.pose_e).onTrue(tower.runOnce(() -> approach.createPathCmd(ApproachTarget.REEF_E)));
        copilot_2.button(DriverConstants.pose_f).onTrue(tower.runOnce(() -> approach.createPathCmd(ApproachTarget.REEF_F)));
        copilot_2.button(DriverConstants.pose_g).onTrue(tower.runOnce(() -> approach.createPathCmd(ApproachTarget.REEF_G)));
        copilot_2.button(DriverConstants.pose_h).onTrue(tower.runOnce(() -> approach.createPathCmd(ApproachTarget.REEF_H)));

              
       
        // reset the field-centric heading on back btn press
        joystick.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        joystick.y().onTrue(drivetrain.runOnce(() -> runCoralStationCmd(true)));
        joystick.b().onTrue(drivetrain.runOnce(() -> runCoralStationCmd(false)));
        
        

        // ==== Approach Buttons ================================
        
        
        joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );
        joystick.pov(90).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0).withVelocityY(-0.5))
        );
        joystick.pov(270).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0).withVelocityY(0.5))
        );
        

        /* 
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        */

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }

    private Command approachCoralStationCommand;
    boolean isAtTarget = false;
    double targetAngle = (approach.tags.getTagPose(12).get().getRotation().toRotation2d().getDegrees());
    double headingError = 0;

    BooleanSupplier atTarget = (() -> {
        
        if(targetAngle - (drivetrain.getState().Pose.getRotation().getDegrees()) > 180){
            headingError = (targetAngle - (drivetrain.getState().Pose.getRotation().getDegrees())) - 360;
        } else if(targetAngle - (drivetrain.getState().Pose.getRotation().getDegrees()) < -180){
            headingError = (targetAngle - (drivetrain.getState().Pose.getRotation().getDegrees())) + 360;
        } else{
            headingError = targetAngle - (drivetrain.getState().Pose.getRotation().getDegrees());
        }

        if(Math.abs(targetAngle - drivetrain.getState().Pose.getRotation().getDegrees()) < 1){
            return true;
        } else{
            
            SmartDashboard.putNumber("Degrees Left to Turn", headingError);
            //System.out.println("Speed: " + clamp((headingError * MaxAngularRate / 120), -Math.PI * 1.5, Math.PI * 1.5, Math.PI/4));
            return false;
        }
    });

    public void faceCoralStation(boolean isLeft){
        int tagId = 13;
        if(!isLeft){
            tagId -= 1;
        }
        approach.alliance = DriverStation.getAlliance();
        if(approach.alliance.get().equals(Alliance.Red)){
            tagId -= 11;
            if(isLeft){
                tagId -= 1;
            } else{
                tagId += 1;
            }
        } else{
            System.out.println(approach.alliance.get());
        }
        targetAngle = (approach.tags.getTagPose(tagId).get().getRotation().toRotation2d().getDegrees());
        /*if(targetAngle > 0){
            targetAngle = targetAngle - 180;
        } else{
            targetAngle = targetAngle + 180;
        }*/
        if(targetAngle - (drivetrain.getState().Pose.getRotation().getDegrees()) > 180){
            headingError = (targetAngle - (drivetrain.getState().Pose.getRotation().getDegrees())) + 360;
        } else{
            headingError = targetAngle - (drivetrain.getState().Pose.getRotation().getDegrees());
        }
        System.out.println("atTarget: " + atTarget.getAsBoolean());
            approachCoralStationCommand = drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed / 2) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed / 2) // Drive left with negative X (left)
                    .withRotationalRate(clamp((headingError * MaxAngularRate / 80), -Math.PI, Math.PI, Math.PI/4)) // Auto rotate to position
            ).until(atTarget); // Run until target angle is reached
      }


    private CommandScheduler scheduler = CommandScheduler.getInstance();

    public void runCoralStationCmd(boolean isLeft) {
        faceCoralStation(isLeft);
        scheduler.schedule(approachCoralStationCommand);
        tower.triggerEvent(TowerEvent.INTAKE_CORAL);
    }

    private double clamp(double value, double min, double max, double innerBound){
        if(value > max){
            value = max;
        } else if(value > 0 && value < innerBound){
            value = innerBound;
        } else if(value < min){
            value = min;
        } else if(value < 0 && value > -innerBound){
            value = -innerBound;
        }
        System.out.println("speed" + value);
        return value;
    }

}
