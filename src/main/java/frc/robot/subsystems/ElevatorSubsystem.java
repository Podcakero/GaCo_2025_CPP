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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  private final SparkFlex m_leftElevatorMotor;
  private SparkFlex m_centerElevatorMotor; // Cannot be final due to sysID Routine method
  private final SparkFlex m_rightElevatorMotor;
  
  private final SparkFlexConfig m_leftElevatorMotorConfig;
  private final SparkFlexConfig m_centerElevatorMotorConfig;
  private final SparkFlexConfig m_rightElevatorMotorConfig;
  private final SparkClosedLoopController m_elevatorController;
  private final RelativeEncoder m_elevatorEncoder;
  private final SparkAnalogSensor m_elevatorAbs;
  
  private final ElevatorFeedforward m_elevatorFeedforward;

	private final TrapezoidProfile m_elevatorTrapezoidProfile;
	private TrapezoidProfile.State m_elevatorGoal = new TrapezoidProfile.State();
	private TrapezoidProfile.State m_elevatorSetpoint;

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutAngle m_distance = Rotations.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutAngularVelocity m_velocity = RotationsPerSecond.mutable(0);

  private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
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
                m_appliedVoltage.mut_replace(
                    m_centerElevatorMotor.get() * RobotController.getBatteryVoltage(), Volts))
            .angularPosition(m_distance.mut_replace(m_centerElevatorMotor.getEncoder().getPosition(), Rotations))
            .angularVelocity(
                m_velocity.mut_replace(m_centerElevatorMotor.getEncoder().getVelocity(), RotationsPerSecond));
      },
      this
    )
  );

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    m_leftElevatorMotor = new SparkFlex(ElevatorConstants.kElevatorMotorLeftId, MotorType.kBrushless);
    m_centerElevatorMotor = new SparkFlex(ElevatorConstants.kElevatorMotorCenterId, MotorType.kBrushless);
    m_rightElevatorMotor = new SparkFlex(ElevatorConstants.kElevatorMotorRightId, MotorType.kBrushless);

    m_elevatorController = m_centerElevatorMotor.getClosedLoopController();
    m_elevatorEncoder = m_centerElevatorMotor.getEncoder();
    m_elevatorAbs = m_centerElevatorMotor.getAnalog();
     
	  m_elevatorFeedforward = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV);

	  m_elevatorTrapezoidProfile = new TrapezoidProfile(new Constraints(ElevatorConstants.kElevatorMaxVelocityRPS,
				                                              ElevatorConstants.kElevatorMaxAccelerationRPSPS));

	  m_elevatorSetpoint = new TrapezoidProfile.State(m_elevatorEncoder.getPosition(), m_elevatorEncoder.getVelocity());

    m_centerElevatorMotorConfig = new SparkFlexConfig();
    m_centerElevatorMotorConfig.closedLoop
      .p(ElevatorConstants.kP)
      .i(ElevatorConstants.kI)
      .d(ElevatorConstants.kD)
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    m_centerElevatorMotorConfig.encoder
      .positionConversionFactor(ElevatorConstants.kElevatorEncoderPositionConversionFactor)
      .velocityConversionFactor(ElevatorConstants.kElevatorEncoderVelocityConversionFactor);
    m_centerElevatorMotorConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(ElevatorConstants.kElevatorCurrentLimit);

    m_leftElevatorMotorConfig = new SparkFlexConfig();
    m_leftElevatorMotorConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(ElevatorConstants.kElevatorCurrentLimit)
      .follow(m_centerElevatorMotor);
    
    m_rightElevatorMotorConfig = new SparkFlexConfig();
    m_rightElevatorMotorConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(ElevatorConstants.kElevatorCurrentLimit)
      .follow(m_centerElevatorMotor);

    m_leftElevatorMotor.configure(m_leftElevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_centerElevatorMotor.configure(m_centerElevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightElevatorMotor.configure(m_rightElevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    setDefaultCommand(runElevatorClosedLoop());
  }

  public Command sysIdQuasistatic(Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  public void setVoltage(Voltage voltage) {
    m_centerElevatorMotor.setVoltage(voltage.in(Volts));
  }

  public Distance getPosition(){
    return Meters.of(m_elevatorEncoder.getPosition());
  }

  public void resetRelativeEncoder() {
    m_elevatorEncoder.setPosition(0.0); // Change 0.0 to instead get the value of the absolute encoder
  }

	public void setGoalPosition(Distance goalPosition) {
	  m_elevatorGoal = new TrapezoidProfile.State(goalPosition.in(Meters), 0.0);
	}

	public void clearGoalPosition() {
		m_elevatorGoal = new TrapezoidProfile.State();
	}

	public void resetSetPoint() {
		m_elevatorSetpoint = new TrapezoidProfile.State(m_elevatorEncoder.getPosition(), 0.0);
	}

  public boolean inPosition(){
    return Math.abs(m_elevatorGoal.position - m_elevatorEncoder.getPosition()) < Constants.ElevatorConstants.kHeightTollerance.in(Meters);
  }

  public Command runElevatorClosedLoop() {
   return Commands.run(() -> {
    m_elevatorSetpoint = m_elevatorTrapezoidProfile.calculate(Constants.kDt, m_elevatorSetpoint, m_elevatorGoal);
  
    double arbFF = m_elevatorFeedforward.calculate(m_elevatorSetpoint.velocity);
      
    m_elevatorController.setReference(
      m_elevatorSetpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, arbFF);
  
    SmartDashboard.putNumber("Elevator FeedForward", arbFF);
   });
  }

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		SmartDashboard.putNumber("ElevatorEncoder", m_elevatorEncoder.getPosition());
		SmartDashboard.putNumber("Elevator Velocity", m_elevatorEncoder.getVelocity() / 60);
		SmartDashboard.putNumber("ElevatorSetpoint", m_elevatorSetpoint.position);
		SmartDashboard.putNumber("ElevatorProfileVelocity", m_elevatorSetpoint.velocity);
		SmartDashboard.putNumber("ElevatorGoal", m_elevatorGoal.position);
		SmartDashboard.putNumber("Elevator Voltage", m_centerElevatorMotor.getAppliedOutput() );
		SmartDashboard.putNumber("Elevator Absolute", m_elevatorAbs.getVoltage());
	}
}
