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
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
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
  private final SparkAnalogSensor elevatorAbs;
  
  private final ElevatorFeedforward elevatorFeedforward;

	private final TrapezoidProfile elevatorTrapezoidProfile;
	private TrapezoidProfile.State elevatorGoal = new TrapezoidProfile.State();
	private TrapezoidProfile.State elevatorSetpoint;

  private double   stringPotVoltage = 0;
  private Distance stringPotHeight = Meters.of(0);
  private Distance relativeEncoderHeight =  Meters.of(0);

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    leftElevatorMotor = new SparkFlex(ElevatorConstants.kElevatorMotorLeftId, MotorType.kBrushless);
    centerElevatorMotor = new SparkFlex(ElevatorConstants.kElevatorMotorCenterId, MotorType.kBrushless);
    rightElevatorMotor = new SparkFlex(ElevatorConstants.kElevatorMotorRightId, MotorType.kBrushless);

    elevatorController = centerElevatorMotor.getClosedLoopController();
    elevatorEncoder = centerElevatorMotor.getEncoder();
    elevatorAbs = centerElevatorMotor.getAnalog();
     
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

    SmartDashboard.putNumber("Elev Abs Volt", stringPotVoltage);
    SmartDashboard.putNumber("Elev Abs Hgt", stringPotHeight.in(Inches));

		SmartDashboard.putNumber("ElevatorGoal", elevatorGoal.position * 39.333);
	}

  public void readSensors() {
    stringPotVoltage  = elevatorAbs.getPosition();
    stringPotHeight   = Meters.of((stringPotVoltage * Constants.ElevatorConstants.kAbsoluteEncoderScaleVoltsToMeters)  + 
                                       Constants.ElevatorConstants.kAbsoluteEncoderOffsetVoltsToMeters); 

    relativeEncoderHeight = Meters.of(elevatorEncoder.getPosition()); 
  }
  
  public void resetElevatorControl() {
    stopElevator();
	  loadCurrentPositionAsSetpoint();
  }

  public void stopElevator() {
    centerElevatorMotor.set(0);
  }

  public void loadCurrentPositionAsSetpoint() {
    setGoalPosition(SyncronizeRelativeEncoder());
	}
  
  public Distance SyncronizeRelativeEncoder() {
    Distance elevatorHeight = stringPotHeight;
    elevatorEncoder.setPosition(elevatorHeight.in(Meters)); 
    return elevatorHeight;
  }
  	
  public void setGoalPosition(Distance goalPosition) {
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
      SmartDashboard.putNumber("Elevator Power", centerElevatorMotor.getAppliedOutput());
  }

  //----------//
  // Commands //
  //----------//
  
}
