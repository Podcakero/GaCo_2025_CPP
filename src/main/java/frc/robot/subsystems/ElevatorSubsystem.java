// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    left = new SparkFlex(ElevatorConstant.kElevatorMotorLeftId, MotorType.kBrushless);
    center = new SparkFlex(ElevatorConstant.kElevatorMotorCenterId, MotorType.kBrushless);
    right = new SparkFlex(ElevatorConstant.kElevatorMotorRightId, MotorType.kBrushless);

    elevatorController = center.getClosedLoopController();
    elevatorEncoder = center.getEncoder();
    pidController = new PIDController(ElevatorConstant.kP, ElevatorConstant.kI, ElevatorConstant.kD);

    centerConfig = new SparkFlexConfig();
   // centerConfig.closedLoop.p(ElevatorConstant.kP).i(ElevatorConstant.kI).d(ElevatorConstant.kD).feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    centerConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(ElevatorConstant.kElevatorCurrentLimit);

    leftConfig = new SparkFlexConfig();
   // leftConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(ElevatorConstant.kElevatorCurrentLimit).follow(center);
    leftConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(ElevatorConstant.kElevatorCurrentLimit);
    
    rightConfig = new SparkFlexConfig();
   // rightConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(ElevatorConstant.kElevatorCurrentLimit).follow(center);
    rightConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(ElevatorConstant.kElevatorCurrentLimit);

    left.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    center.configure(centerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    right.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setPosition(double position){
    //elevatorController.setReference(position, ControlType.kPosition);
    left.set(pidController.calculate(elevatorEncoder.getPosition(), position));
    center.set(pidController.calculate(elevatorEncoder.getPosition(), position));
    right.set(pidController.calculate(elevatorEncoder.getPosition(), position));
  }

  public double getPosition(){
    return elevatorEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
