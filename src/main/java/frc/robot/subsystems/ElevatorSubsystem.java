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
import frc.robot.Constants.Elevator;
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
  private Distance lastGoalPosition = Constants.Elevator.kElevatorMinHeight;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    leftElevatorMotor = new SparkFlex(Elevator.kElevatorMotorLeftId, MotorType.kBrushless);
    centerElevatorMotor = new SparkFlex(Elevator.kElevatorMotorCenterId, MotorType.kBrushless);
    rightElevatorMotor = new SparkFlex(Elevator.kElevatorMotorRightId, MotorType.kBrushless);

    elevatorController = centerElevatorMotor.getClosedLoopController();
    elevatorEncoder = centerElevatorMotor.getEncoder();
  
	  elevatorFeedforward = new ElevatorFeedforward(Elevator.kS, Elevator.kG, Elevator.kV);

	  elevatorTrapezoidProfile = new TrapezoidProfile(new Constraints(Elevator.kElevatorMaxVelocityRPS,
				                                              Elevator.kElevatorMaxAccelerationRPSPS));

	  elevatorSetpoint = new TrapezoidProfile.State(elevatorEncoder.getPosition(), elevatorEncoder.getVelocity());

    centerElevatorMotorConfig = new SparkFlexConfig();

    centerElevatorMotorConfig.closedLoop
      .p(Elevator.kP)
      .i(Elevator.kI)
      .d(Elevator.kD)
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    centerElevatorMotorConfig.encoder
      .positionConversionFactor(Elevator.kElevatorEncoderPositionConversionFactor)
      .velocityConversionFactor(Elevator.kElevatorEncoderVelocityConversionFactor);

    centerElevatorMotorConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(Elevator.kElevatorCurrentLimit);

    leftElevatorMotorConfig = new SparkFlexConfig();
    leftElevatorMotorConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(Elevator.kElevatorCurrentLimit)
      .follow(centerElevatorMotor);
    
    rightElevatorMotorConfig = new SparkFlexConfig();
    rightElevatorMotorConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(Elevator.kElevatorCurrentLimit)
      .follow(centerElevatorMotor);

    leftElevatorMotor.configure(leftElevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    centerElevatorMotor.configure(centerElevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightElevatorMotor.configure(rightElevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    setDefaultCommand(new DefaultElevatorCmd(this));

    elevatorEncoder.setPosition(Constants.Elevator.elevatorHomeHeight.in(Meters)); 


  }

    /**
   * WARNING: This will rebase the elevator at the current position.
   */
  public void resetEncoder(){
    elevatorEncoder.setPosition(Constants.Elevator.elevatorHomeHeight.in(Meters)); 
    initialize();
  }

  public void initialize(){
    readSensors();
    resetElevatorControl();
  }

  public void resetFrameRate() {
    SparkFlexConfig config = new SparkFlexConfig();
    config.signals.appliedOutputPeriodMs(10);
    leftElevatorMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    centerElevatorMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    rightElevatorMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    System.out.println("RESET FRAME RATE");
  }


  @Override
	public void periodic() {

    readSensors();

		// This method will be called once per scheduler run
    SmartDashboard.putNumber("Elev Rel Hgt", relativeEncoderHeight.in(Inches));
		SmartDashboard.putNumber("ElevatorGoal", elevatorGoal.position * 39.333);
    SmartDashboard.putNumber("Elevator Power", centerElevatorMotor.getAppliedOutput());
    SmartDashboard.putBoolean("Elevator In Position", inPosition());
	}

  public void readSensors() {
    getCurrent();
    relativeEncoderHeight = Meters.of(elevatorEncoder.getPosition()); 
  }
  
  public void bumpElevator(double changeMeters) {
    setGoalPosition(Meters.of(lastGoalPosition.in(Meters) + changeMeters));
  }

  public void resetElevatorControl() {
    setGoalPosition(relativeEncoderHeight);
  }

  public void setGoalPosition(Distance goalPosition) {
    if (goalPosition.lt(Constants.Elevator.kElevatorMinHeight)) {
      goalPosition = Constants.Elevator.kElevatorMinHeight;
    } else if (goalPosition.gt(Constants.Elevator.kElevatorMaxHeight)) {
      goalPosition = Constants.Elevator.kElevatorMaxHeight;
    }

    lastGoalPosition = goalPosition;
	  elevatorGoal = new TrapezoidProfile.State(goalPosition.in(Meters), 0.0);
    elevatorSetpoint = new TrapezoidProfile.State(elevatorEncoder.getPosition(), 0.0);
	}

  public Distance getHeight(){
    return relativeEncoderHeight;
  }

  public boolean inPosition(){
    Globals.ELEVATOR_IN_POSITION = (Math.abs(elevatorGoal.position - elevatorEncoder.getPosition()) < Constants.Elevator.kHeightTollerance.in(Meters));
    return Globals.ELEVATOR_IN_POSITION;
  }
	
  public void runClosedLoop() {
    elevatorSetpoint = elevatorTrapezoidProfile.calculate(Constants.kDt, elevatorSetpoint, elevatorGoal);
  
    double arbFF = elevatorFeedforward.calculate(elevatorSetpoint.velocity);
      
    elevatorController.setReference(elevatorSetpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, arbFF);
  
    SmartDashboard.putNumber("Elevator FeedForward", arbFF);
  }

  public void setSpeed(double speed) {
    centerElevatorMotor.set(speed);
  }

  public double getCurrent() {
    double current = leftElevatorMotor.getOutputCurrent() +  centerElevatorMotor.getOutputCurrent() + rightElevatorMotor.getOutputCurrent();
    SmartDashboard.putNumber("Elevator Current", current);
    return current;
  }

  //----------//
  // Commands //
  //----------//
  
}
