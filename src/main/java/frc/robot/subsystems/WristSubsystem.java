// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WristSubsystem extends SubsystemBase {

  private final SparkMax m_drivingSpark;
  private final SparkMax m_turningSpark;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SparkClosedLoopController m_drivingClosedLoopController;
  private final SparkClosedLoopController m_turningClosedLoopController;
  private SparkFlex wristMotor;
  private RelativeEncoder wristEncoder;

  /** Creates a new WristSubsystem. */
  public WristSubsystem() {

    m_drivingSpark = new SparkMax(Constants.WristConstants.kIntakeMotorId, MotorType.kBrushless);
    m_turningSpark = new SparkMax(Constants.WristConstants.kWristMotorId, MotorType.kBrushless);

    m_drivingEncoder = m_drivingSpark.getEncoder();
    m_turningEncoder = m_turningSpark.getAbsoluteEncoder();

    m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
    m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

    double wristAngularOffset = 0;
  

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.

    SparkMaxConfig intakeConfig = new SparkMaxConfig();
    SparkMaxConfig angleConfig = new SparkMaxConfig();

                // Use module constants to calculate conversion factors and feed forward gain.
    double intakeFactor = 1;
    double turningFactor = 1;

    intakeConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(50);
    intakeConfig.encoder
            .positionConversionFactor(intakeFactor) // meters
            .velocityConversionFactor(intakeFactor / 60.0); // meters per second
    
    angleConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(20);
    angleConfig.absoluteEncoder
            .inverted(true)
            .positionConversionFactor(turningFactor) // radians
            .velocityConversionFactor(turningFactor / 60.0); // radians per second
    angleConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            // These are example gains you may need to them for your own robot!
            .pid(1, 0, 0)
            .outputRange(-1, 1)
            .positionWrappingInputRange(0, turningFactor);

    m_drivingSpark.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_turningSpark.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_drivingEncoder.setPosition(0);
  }

  public void setIntakeSpeed(double speed) {

  }


  public double getAngle(){
    return wristEncoder.getPosition();
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("Wrist Position", wristEncoder.getPosition());
    //SmartDashboard.putNumber("Wrist Velocity", wristEncoder.getVelocity());
  }
}
