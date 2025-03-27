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
import frc.robot.Constants.Elevator;
import frc.robot.commands.DefaultElevatorCmd;

public class ElevatorSubsystem extends SubsystemBase {

  private final boolean   USING_THREE_MOTORS = true;

  private final SparkFlex leaderElevatorMotor; // Cannot be final due to sysID Routine method
  private final SparkFlex follower1ElevatorMotor;
  private final SparkFlex follower2ElevatorMotor;
  
  private final SparkFlexConfig leaderElevatorMotorConfig;
  private final SparkFlexConfig follower1ElevatorMotorConfig;
  private final SparkFlexConfig follower2ElevatorMotorConfig;
  
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
    leaderElevatorMotor     = new SparkFlex(Elevator.kElevatorMotorLeftId, MotorType.kBrushless);
    follower1ElevatorMotor  = new SparkFlex(Elevator.kElevatorMotorRightId, MotorType.kBrushless);
    
    if (USING_THREE_MOTORS) {
      follower2ElevatorMotor= new SparkFlex(Elevator.kElevatorMotorCenterId, MotorType.kBrushless);
    } else {
      follower2ElevatorMotor = null;
    }

    elevatorController = leaderElevatorMotor.getClosedLoopController();
    elevatorEncoder = leaderElevatorMotor.getEncoder();
  
	  elevatorFeedforward = new ElevatorFeedforward(Elevator.kS, Elevator.kG, Elevator.kV);

	  elevatorTrapezoidProfile = new TrapezoidProfile(new Constraints(Elevator.kElevatorMaxVelocityMPS,
				                                              Elevator.kElevatorMaxAccelerationMPSPS));

	  elevatorSetpoint = new TrapezoidProfile.State(elevatorEncoder.getPosition(), elevatorEncoder.getVelocity());

    leaderElevatorMotorConfig = new SparkFlexConfig();

    leaderElevatorMotorConfig.closedLoop
      .p(Elevator.kP)
      .i(Elevator.kI)
      .d(Elevator.kD)
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    leaderElevatorMotorConfig.encoder
      .positionConversionFactor(Elevator.kElevatorEncoderPositionConversionFactor)
      .velocityConversionFactor(Elevator.kElevatorEncoderVelocityConversionFactor);

    leaderElevatorMotorConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(Elevator.kElevatorCurrentLimit);

    follower1ElevatorMotorConfig = new SparkFlexConfig();
    follower1ElevatorMotorConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(Elevator.kElevatorCurrentLimit)
      .follow(leaderElevatorMotor);
    
    leaderElevatorMotor.configure(leaderElevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    follower1ElevatorMotor.configure(follower1ElevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    if (USING_THREE_MOTORS){
      follower2ElevatorMotorConfig = new SparkFlexConfig();
      follower2ElevatorMotorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Elevator.kElevatorCurrentLimit)
        .follow(leaderElevatorMotor);
      follower2ElevatorMotor.configure(follower2ElevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    } else {
      follower2ElevatorMotorConfig = null;
    }
    
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
    leaderElevatorMotor.configure(resetFrameRateConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    follower1ElevatorMotor.configure(resetFrameRateConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    if (USING_THREE_MOTORS){
      follower2ElevatorMotor.configure(resetFrameRateConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
    System.out.println("RESET FRAME RATE");
  }

  @Override
  public void simulationPeriodic() {
      SmartDashboard.putNumber("Elev Rel Hgt", Units.metersToInches(elevatorGoal.position));
      SmartDashboard.putNumber("Elevator Goal", Units.metersToInches(elevatorGoal.position));
      SmartDashboard.putString("Elevator Power", "SIMULATION");
  }

  @Override
	public void periodic() {
    readSensors();

		// This method will be called once per scheduler run
    SmartDashboard.putNumber("Elev Rel Hgt", Units.metersToInches(relativeEncoderHeightMeters));
		SmartDashboard.putNumber("Elevator Goal", Units.metersToInches(elevatorGoal.position));
    SmartDashboard.putNumber("Elevator Power", leaderElevatorMotor.getAppliedOutput());
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
    leaderElevatorMotor.set(speed);
  }

  public double getCurrent() {
    if (USING_THREE_MOTORS){
      return leaderElevatorMotor.getOutputCurrent() +  follower1ElevatorMotor.getOutputCurrent() + follower2ElevatorMotor.getOutputCurrent();
    } else {  
      return leaderElevatorMotor.getOutputCurrent() +  follower1ElevatorMotor.getOutputCurrent() ;
    }
  }

  //----------//
  // Commands //
  //----------//
  
}
