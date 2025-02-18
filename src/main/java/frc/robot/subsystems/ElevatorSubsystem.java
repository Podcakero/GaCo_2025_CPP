// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
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
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  private SparkFlex m_leftElevatorMotor;
  private SparkFlex m_centerElevatorMotor;
  private SparkFlex m_rightElevatorMotor;
  
  private SparkFlexConfig m_leftElevatorMotorConfig;
  private SparkFlexConfig m_centerElevatorMotorConfig;
  private SparkFlexConfig m_rightElevatorMotorConfig;

  private SparkClosedLoopController m_elevatorController;
  private RelativeEncoder m_elevatorEncoder;

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
    //pidController = new PIDController(ElevatorConstant.kP, ElevatorConstant.kI, ElevatorConstant.kD);

    m_centerElevatorMotorConfig = new SparkFlexConfig();
    m_centerElevatorMotorConfig.closedLoop.p(ElevatorConstants.kP).i(ElevatorConstants.kI).d(ElevatorConstants.kD).feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    m_centerElevatorMotorConfig.encoder.positionConversionFactor(ElevatorConstants.kElevatorEncoderPositionConversionFactor).velocityConversionFactor(ElevatorConstants.kElevatorEncoderVelocityConversionFactor);
    m_centerElevatorMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(ElevatorConstants.kElevatorCurrentLimit);

    m_leftElevatorMotorConfig = new SparkFlexConfig();
    m_leftElevatorMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(ElevatorConstants.kElevatorCurrentLimit).follow(m_centerElevatorMotor);
    //leftConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(ElevatorConstant.kElevatorCurrentLimit);
    
    m_rightElevatorMotorConfig = new SparkFlexConfig();
    m_rightElevatorMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(ElevatorConstants.kElevatorCurrentLimit).follow(m_centerElevatorMotor);
    //rightConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(ElevatorConstant.kElevatorCurrentLimit);

    m_leftElevatorMotor.configure(m_leftElevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_centerElevatorMotor.configure(m_centerElevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightElevatorMotor.configure(m_rightElevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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

  public void setPosition(Distance position){
    m_elevatorController.setReference(position.in(Meters), ControlType.kPosition);
    //left.set(pidController.calculate(elevatorEncoder.getPosition(), position));
    //center.set(pidController.calculate(elevatorEncoder.getPosition(), position));
    //right.set(pidController.calculate(elevatorEncoder.getPosition(), position));
  }

  public Distance getPosition(){
    return Meters.of(m_elevatorEncoder.getPosition());
  }

  public void resetRelativeEncoder() {
    m_elevatorEncoder.setPosition(0.0); // Change 0.0 to instead get the value of the absolute encoder
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ElevatorEncoder", m_elevatorEncoder.getPosition());
  }
}
