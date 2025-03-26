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
import edu.wpi.first.math.util.Units;

import com.ctre.phoenix6.Utils;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.Elevator;
import frc.robot.commands.DefaultElevatorCmd;

public class ElevatorSubsystem extends SubsystemBase {
  private final SparkFlex leftElevatorMotor;
  private final SparkFlex centerElevatorMotor; // Cannot be final due to sysID Routine method
  private final SparkFlex rightElevatorMotor;
  
  private final SparkFlexConfig leftElevatorMotorConfig;
  private final SparkFlexConfig centerElevatorMotorConfig;
  private final SparkFlexConfig rightElevatorMotorConfig;
  private final SparkFlexConfig resetFrameRateConfig;
  private final SparkClosedLoopController elevatorController;
  private final RelativeEncoder elevatorEncoder;

  private final ElevatorFeedforward elevatorFeedforward;

	private final TrapezoidProfile elevatorTrapezoidProfile;
	private TrapezoidProfile.State elevatorGoal = new TrapezoidProfile.State();
	private TrapezoidProfile.State elevatorSetpoint;

  private double relativeEncoderHeightMeters =  0;
  private double lastGoalPositionMeters = Constants.Elevator.kElevatorMinHeightMeters;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    leftElevatorMotor = new SparkFlex(Elevator.kElevatorMotorLeftId, MotorType.kBrushless);
    centerElevatorMotor = new SparkFlex(Elevator.kElevatorMotorCenterId, MotorType.kBrushless);
    rightElevatorMotor = new SparkFlex(Elevator.kElevatorMotorRightId, MotorType.kBrushless);

    elevatorController = centerElevatorMotor.getClosedLoopController();
    elevatorEncoder = centerElevatorMotor.getEncoder();
  
	  elevatorFeedforward = new ElevatorFeedforward(Elevator.kS, Elevator.kG, Elevator.kV);

	  elevatorTrapezoidProfile = new TrapezoidProfile(new Constraints(elevatorFeedforward.maxAchievableVelocity(12.0, Elevator.kElevatorMaxAccelerationMPSPS),
				                                              Elevator.kElevatorMaxAccelerationMPSPS));

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

    elevatorEncoder.setPosition(Constants.Elevator.elevatorHomeHeightMeters); 

    resetFrameRateConfig = new SparkFlexConfig();
    resetFrameRateConfig.signals.appliedOutputPeriodMs(10);
  }

    /**
   * WARNING: This will rebase the elevator at the current position.
   */
  public void resetEncoder(){
    elevatorEncoder.setPosition(Constants.Elevator.elevatorHomeHeightMeters); 
    initialize();
  }

  public void initialize(){
    readSensors();
    resetElevatorControl();
  }

  public void resetFrameRate() {
    leftElevatorMotor.configure(resetFrameRateConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    centerElevatorMotor.configure(resetFrameRateConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    rightElevatorMotor.configure(resetFrameRateConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    System.out.println("RESET FRAME RATE");
  }

  @Override
  public void simulationPeriodic() {
      // TODO Auto-generated method stub
      SmartDashboard.putNumber("Elev Rel Hgt", Units.metersToInches(elevatorGoal.position));
      SmartDashboard.putNumber("ElevatorGoal", Units.metersToInches(elevatorGoal.position));
      SmartDashboard.putString("Elevator Power", "SIMULATION");
  }

  @Override
	public void periodic() {
    readSensors();

		// This method will be called once per scheduler run
    SmartDashboard.putNumber("Elev Rel Hgt", Units.metersToInches(relativeEncoderHeightMeters));
		SmartDashboard.putNumber("Elevator Goal", Units.metersToInches(elevatorGoal.position));
    SmartDashboard.putNumber("Elevator Power", centerElevatorMotor.getAppliedOutput());
    SmartDashboard.putNumber("Elevator Current", getCurrent());
	}

  public void readSensors() {
    getCurrent();
    relativeEncoderHeightMeters = elevatorEncoder.getPosition(); 
  }
  
  public void bumpElevatorMeters(double changeMeters) {
    setGoalPositionMeters(lastGoalPositionMeters + changeMeters);
  }

  public void resetElevatorControl() {
    setGoalPositionMeters(relativeEncoderHeightMeters);
  }

  public void setGoalPositionMeters(double goalPositionMeters) {
    if (goalPositionMeters < Constants.Elevator.kElevatorMinHeightMeters) {
      goalPositionMeters = Constants.Elevator.kElevatorMinHeightMeters;
    } else if (goalPositionMeters > Constants.Elevator.kElevatorMaxHeightMeters) {
      goalPositionMeters = Constants.Elevator.kElevatorMaxHeightMeters;
    }

    lastGoalPositionMeters = goalPositionMeters;
    
	  elevatorGoal.position = goalPositionMeters;
    elevatorGoal.velocity = 0.0;

    elevatorSetpoint.position = elevatorEncoder.getPosition();
    elevatorSetpoint.velocity = 0.0;
	}

  public double getHeightMeters(){
    return relativeEncoderHeightMeters;
  }

  public boolean inPosition(){
    if (Utils.isSimulation()){
      Globals.ELEVATOR_IN_POSITION = true;
      return Globals.ELEVATOR_IN_POSITION;
    } else {
      Globals.ELEVATOR_IN_POSITION = (Math.abs(elevatorGoal.position - elevatorEncoder.getPosition()) < Constants.Elevator.kHeightTolleranceMeters);
      return Globals.ELEVATOR_IN_POSITION;
    }
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
    return leftElevatorMotor.getOutputCurrent() +  centerElevatorMotor.getOutputCurrent() + rightElevatorMotor.getOutputCurrent();
  }

  //----------//
  // Commands //
  //----------//
  
}
