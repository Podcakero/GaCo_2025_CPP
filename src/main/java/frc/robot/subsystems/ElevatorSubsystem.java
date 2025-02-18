// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ElevatorConstant;

public class ElevatorSubsystem extends SubsystemBase {
  SparkFlex left;
  SparkFlex center;
  SparkFlex right;
  
  SparkFlexConfig leftConfig;
  SparkFlexConfig centerConfig;
  SparkFlexConfig rightConfig;

  SparkClosedLoopController elevatorController;
  RelativeEncoder elevatorEncoder;
  PIDController pidController;

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutAngle m_distance = Rotations.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutAngularVelocity m_velocity = RotationsPerSecond.mutable(0);

  private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
    new SysIdRoutine.Config(
      Volts.of(1).per(Seconds),
      Volts.of(4),
      Seconds.of(4),
      null
    ),
    new SysIdRoutine.Mechanism(
      output -> setVoltage(output.in(Volts)),
      log -> {
        // Record a frame for the left motors.  Since these share an encoder, we consider
        // the entire group to be one motor.
        log.motor("elevator")
            .voltage(
                m_appliedVoltage.mut_replace(
                    center.get() * RobotController.getBatteryVoltage(), Volts))
            .angularPosition(m_distance.mut_replace(center.getEncoder().getPosition(), Rotations))
            .angularVelocity(
                m_velocity.mut_replace(center.getEncoder().getVelocity() / 60, RotationsPerSecond));
      },
      this
    )
    );

  private ShuffleboardTab elevatorTab = Shuffleboard.getTab("Elevator");

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    left = new SparkFlex(ElevatorConstant.kElevatorMotorLeftId, MotorType.kBrushless);
    center = new SparkFlex(ElevatorConstant.kElevatorMotorCenterId, MotorType.kBrushless);
    right = new SparkFlex(ElevatorConstant.kElevatorMotorRightId, MotorType.kBrushless);

    elevatorController = center.getClosedLoopController();
    elevatorEncoder = center.getEncoder();
    //pidController = new PIDController(ElevatorConstant.kP, ElevatorConstant.kI, ElevatorConstant.kD);

    centerConfig = new SparkFlexConfig();
    centerConfig.closedLoop.p(ElevatorConstant.kP).i(ElevatorConstant.kI).d(ElevatorConstant.kD).feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    centerConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(ElevatorConstant.kElevatorCurrentLimit);

    leftConfig = new SparkFlexConfig();
    leftConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(ElevatorConstant.kElevatorCurrentLimit).follow(center);
    //leftConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(ElevatorConstant.kElevatorCurrentLimit);
    
    rightConfig = new SparkFlexConfig();
    rightConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(ElevatorConstant.kElevatorCurrentLimit).follow(center);
    //rightConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(ElevatorConstant.kElevatorCurrentLimit);

    elevatorEncoder.setPosition(0.0);

    left.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    center.configure(centerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    right.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public Command sysIdQuasistatic(Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  public void setVoltage(double voltage) {
    center.setVoltage(voltage);
  }

  public void setPosition(double position){
    elevatorController.setReference(position, ControlType.kPosition);
    //left.set(pidController.calculate(elevatorEncoder.getPosition(), position));
    //center.set(pidController.calculate(elevatorEncoder.getPosition(), position));
    //right.set(pidController.calculate(elevatorEncoder.getPosition(), position));
  }

  public double getPosition(){
    return elevatorEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ElevatorEncoder", elevatorEncoder.getPosition());
  }
}
