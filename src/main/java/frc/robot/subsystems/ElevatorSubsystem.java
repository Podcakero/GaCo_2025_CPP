// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.DefaultElevatorCmd;

public class ElevatorSubsystem extends SubsystemBase {
  private final SparkFlex leftElevatorMotor;
  private SparkFlex centerElevatorMotor; // Cannot be final due to sysID Routine method
  private final SparkFlex rightElevatorMotor;
  
  private final SparkFlexConfig leftElevatorMotorConfig;
  private final SparkFlexConfig centerElevatorMotorConfig;
  private final SparkFlexConfig rightElevatorMotorConfig;
  private final SparkClosedLoopController elevatorController;
  private final RelativeEncoder elevatorEncoder;
  
  private final ElevatorFeedforward elevatorFeedforward;

	private final TrapezoidProfile elevatorTrapezoidProfile;
	private TrapezoidProfile.State elevatorGoal = new TrapezoidProfile.State();
	private TrapezoidProfile.State elevatorSetpoint;

  private Distance relativeEncoderHeight =  Meters.of(0);
  private Distance lastGoalPosition = Constants.ElevatorConstants.kElevatorMinHeight;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    leftElevatorMotor = new SparkFlex(ElevatorConstants.kElevatorMotorLeftId, MotorType.kBrushless);
    centerElevatorMotor = new SparkFlex(ElevatorConstants.kElevatorMotorCenterId, MotorType.kBrushless);
    rightElevatorMotor = new SparkFlex(ElevatorConstants.kElevatorMotorRightId, MotorType.kBrushless);

    elevatorController = centerElevatorMotor.getClosedLoopController();
    elevatorEncoder = centerElevatorMotor.getEncoder();
  
	  elevatorFeedforward = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV);

	  elevatorTrapezoidProfile = new TrapezoidProfile(new Constraints(ElevatorConstants.kElevatorMaxVelocityRPS,
				                                              ElevatorConstants.kElevatorMaxAccelerationRPSPS));

	  elevatorSetpoint = new TrapezoidProfile.State(elevatorEncoder.getPosition(), elevatorEncoder.getVelocity());

    centerElevatorMotorConfig = new SparkFlexConfig();

    centerElevatorMotorConfig.closedLoop
      .p(ElevatorConstants.kP)
      .i(ElevatorConstants.kI)
      .d(ElevatorConstants.kD)
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    centerElevatorMotorConfig.encoder
      .positionConversionFactor(ElevatorConstants.kElevatorEncoderPositionConversionFactor)
      .velocityConversionFactor(ElevatorConstants.kElevatorEncoderVelocityConversionFactor);

    centerElevatorMotorConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(ElevatorConstants.kElevatorCurrentLimit);

    leftElevatorMotorConfig = new SparkFlexConfig();
    leftElevatorMotorConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(ElevatorConstants.kElevatorCurrentLimit)
      .follow(centerElevatorMotor);
    
    rightElevatorMotorConfig = new SparkFlexConfig();
    rightElevatorMotorConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(ElevatorConstants.kElevatorCurrentLimit)
      .follow(centerElevatorMotor);

    leftElevatorMotor.configure(leftElevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    centerElevatorMotor.configure(centerElevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightElevatorMotor.configure(rightElevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    setDefaultCommand(new DefaultElevatorCmd(this));

    Distance elevatorHeight = Inches.of(17.5);  // only valid when elevator is homed;
    elevatorEncoder.setPosition(elevatorHeight.in(Meters)); 
  }

  public void initialize(){
    readSensors();
    resetElevatorControl();
  }

  @Override
	public void periodic() {

    readSensors();

		// This method will be called once per scheduler run
    SmartDashboard.putNumber("Elev Rel Hgt", relativeEncoderHeight.in(Inches));
		SmartDashboard.putNumber("ElevatorGoal", elevatorGoal.position * 39.333);
    SmartDashboard.putNumber("Elevator Power", centerElevatorMotor.getAppliedOutput());
	}

  public void readSensors() {
    relativeEncoderHeight = Meters.of(elevatorEncoder.getPosition()); 
  }
  
  public void bumpElevator(double changeMeters) {
    setGoalPosition(Meters.of(lastGoalPosition.in(Meters) + changeMeters));
  }

  public void resetElevatorControl() {
    setGoalPosition(relativeEncoderHeight);
  }

  public void setGoalPosition(Distance goalPosition) {
    if (goalPosition.lt(Constants.ElevatorConstants.kElevatorMinHeight)) {
      goalPosition = Constants.ElevatorConstants.kElevatorMinHeight;
    } else if (goalPosition.gt(Constants.ElevatorConstants.kElevatorMaxHeight)) {
      goalPosition = Constants.ElevatorConstants.kElevatorMaxHeight;
    }

    lastGoalPosition = goalPosition;
	  elevatorGoal = new TrapezoidProfile.State(goalPosition.in(Meters), 0.0);
    elevatorSetpoint = new TrapezoidProfile.State(elevatorEncoder.getPosition(), 0.0);
	}

  public Distance getHeight(){
    return relativeEncoderHeight;
  }

  public boolean inPosition(){
    return Math.abs(elevatorGoal.position - elevatorEncoder.getPosition()) < Constants.ElevatorConstants.kHeightTollerance.in(Meters);
  }
	
  public void runClosedLoop() {
    elevatorSetpoint = elevatorTrapezoidProfile.calculate(Constants.kDt, elevatorSetpoint, elevatorGoal);
  
    double arbFF = elevatorFeedforward.calculate(elevatorSetpoint.velocity);
      
    elevatorController.setReference(
      elevatorSetpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, arbFF);
  
      SmartDashboard.putNumber("Elevator FeedForward", arbFF);
      
  }

  //----------//
  // Commands //
  //----------//
  
}
