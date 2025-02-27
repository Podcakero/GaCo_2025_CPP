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

import static edu.wpi.first.units.Units.Meters;
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
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
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

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutVoltage appliedVoltage = Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutAngle distance = Rotations.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutAngularVelocity velocity = RotationsPerSecond.mutable(0);

  private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
    new SysIdRoutine.Config(
      Volts.of(0.5).per(Seconds),
      Volts.of(2),
      Seconds.of(5),
      null
    ),
    new SysIdRoutine.Mechanism(
      output -> setVoltage(output),
      log -> {
        // Record a frame for the left motors.  Since these share an encoder, we consider
        // the entire group to be one motor.
        log.motor("elevator")
            .voltage(
                appliedVoltage.mut_replace(
                    centerElevatorMotor.get() * RobotController.getBatteryVoltage(), Volts))
            .angularPosition(distance.mut_replace(centerElevatorMotor.getEncoder().getPosition(), Rotations))
            .angularVelocity(
                velocity.mut_replace(centerElevatorMotor.getEncoder().getVelocity(), RotationsPerSecond));
      },
      this
    )
  );

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

  public Command sysIdQuasistatic(Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  public void resetElevatorControl() {
	  elevatorGoal = new TrapezoidProfile.State(getPosition().in(Meters), 0.0);
    resetSetPoint();
  }

  public void stopElevator() {
    centerElevatorMotor.set(0);
  }

  public void setVoltage(Voltage voltage) {
    centerElevatorMotor.setVoltage(voltage.in(Volts));
  }

  public Distance getPosition(){
    return Meters.of(elevatorEncoder.getPosition());
  }

  public void resetRelativeEncoder() {
    elevatorEncoder.setPosition(0.0); // Change 0.0 to instead get the value of the absolute encoder
  }

	public void setGoalPosition(Distance goalPosition) {
	  elevatorGoal = new TrapezoidProfile.State(goalPosition.in(Meters), 0.0);
	}

	//public void clearGoalPosition() {     /// This seems dangerous to me.   
	//	elevatorGoal = new TrapezoidProfile.State();
	//}

	public void resetSetPoint() {
		elevatorSetpoint = new TrapezoidProfile.State(elevatorEncoder.getPosition(), 0.0);
	}

  public boolean inPosition(){
    return Math.abs(elevatorGoal.position - elevatorEncoder.getPosition()) < Constants.ElevatorConstants.kHeightTollerance.in(Meters);
  }

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		SmartDashboard.putNumber("ElevatorEncoder", elevatorEncoder.getPosition());
		SmartDashboard.putNumber("Elevator Velocity", elevatorEncoder.getVelocity() / 60);
		SmartDashboard.putNumber("ElevatorSetpoint", elevatorSetpoint.position);
		SmartDashboard.putNumber("ElevatorProfileVelocity", elevatorSetpoint.velocity);
		SmartDashboard.putNumber("ElevatorGoal", elevatorGoal.position);
		SmartDashboard.putNumber("Elevator Voltage", centerElevatorMotor.getAppliedOutput() );
		SmartDashboard.putNumber("Elevator Absolute", elevatorAbs.getVoltage());
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
