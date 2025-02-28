// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.DefaultWristCmd;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

public class WristSubsystem extends SubsystemBase {

  private final SparkFlex intakeSpark;
  private final SparkFlex angleSpark;

  private final RelativeEncoder intakeEncoder;
  private final AbsoluteEncoder angleEncoder;

  private final SparkClosedLoopController angleController;
  private final TrapezoidProfile angleTrapezoidProfile;
	private TrapezoidProfile.State angleGoal = new TrapezoidProfile.State();
	private TrapezoidProfile.State angleSetpoint;

  TimeOfFlight TOF;
  double frontCoralRange = 0;
  
  /** Creates a new WristSubsystem. */
  public WristSubsystem() {

    intakeSpark = new SparkFlex(Constants.WristConstants.kIntakeMotorId, MotorType.kBrushless);
    angleSpark = new SparkFlex(Constants.WristConstants.kAngleMotorId, MotorType.kBrushless);

    intakeEncoder = intakeSpark.getEncoder();
    angleEncoder = angleSpark.getAbsoluteEncoder();

    angleTrapezoidProfile = new TrapezoidProfile(new Constraints(Constants.WristConstants.kAngleMaxVelocityDPS,
				                                              Constants.WristConstants.kAngleMaxAccelerationDPSPS));

	  angleSetpoint = new TrapezoidProfile.State(angleEncoder.getPosition(), angleEncoder.getVelocity());
    angleController = angleSpark.getClosedLoopController();

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.

    SparkFlexConfig intakeConfig = new SparkFlexConfig();
    SparkFlexConfig angleConfig = new SparkFlexConfig();

    // Use module constants to calculate conversion factors and feed forward gain.
    double intakeFactor = 1;
    double angleFactor = 360 * 24 / 52;  // Sprocket reduction


    intakeConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(50);
    intakeConfig.encoder
      .positionConversionFactor(intakeFactor) // meters
      .velocityConversionFactor(intakeFactor / 60.0); // meters per second
    
    angleConfig
      .idleMode(IdleMode.kBrake)
      //.inverted(true)
      .smartCurrentLimit(20);
    angleConfig.absoluteEncoder
      //.inverted(true)
      .positionConversionFactor(angleFactor) // degrees
      .velocityConversionFactor(angleFactor / 60.0); // degrees per second
    angleConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      // These are example gains you may need to them for your own robot!
      .pid(Constants.WristConstants.kP, Constants.WristConstants.kI, Constants.WristConstants.kD)
      .outputRange(-Constants.WristConstants.kAnglePower, Constants.WristConstants.kAnglePower)
      .positionWrappingInputRange(0, angleFactor);

    intakeSpark.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    angleSpark.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    TOF = new TimeOfFlight(63);
    TOF.setRangingMode(RangingMode.Short, 30);
    TOF.setRangeOfInterest(0, 0, 15, 15);

    setDefaultCommand( new DefaultWristCmd(this));
  }


  // The configuration interfaces may be accessed by typing in the IP address of the roboRIO into a web
  //  browser followed by :5812.

  // TOF sensor
  public void setViewZone(int topLeftX, int topLeftY, int bottomRightX, int bottomRightY) {
    TOF.setRangeOfInterest(topLeftX, topLeftY, bottomRightX, bottomRightY);
  }

  public boolean gotCoral() {
    Globals.gotCoral = (frontCoralRange < Constants.WristConstants.kMaxCoralDetectRangeMM); 
    return Globals.gotCoral;
  }

  public double getRangeMM() {
    frontCoralRange = TOF.getRange();
    return frontCoralRange ;
  }
      
  // intake
  public void setIntakeSpeed(double speed) {
    intakeSpark.set(speed);
  }

  public double getIntakeSpeed() {
    return intakeEncoder.getVelocity();
  }

  // wrist
  public void resetWristControl () {
    angleGoal = new TrapezoidProfile.State(getWristAngle(), 0.0);
    angleSetpoint = new TrapezoidProfile.State(getWristAngle(), 0.0);
  }

  public void setGoalAngle(double angle){
    angleGoal = new TrapezoidProfile.State(angle, 0.0);
  }

  public double getWristAngle(){
    return angleEncoder.getPosition();
  }

  public double getWristSpeed(){
    return angleEncoder.getVelocity();
  }

  public void stopWrist() {
    angleSpark.set(0);
  }

  public boolean inPosition(){
    return Math.abs(angleGoal.position - getWristAngle()) < Constants.WristConstants.kAngleTollerance;
  }

  @Override
  public void periodic() {

    getRangeMM();
    gotCoral();

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Wrist Goal", angleGoal.position);    
    SmartDashboard.putNumber("Wrist Angle", getWristAngle());
    SmartDashboard.putNumber("Wrist Speed", getWristSpeed());
    SmartDashboard.putNumber("Wrist Power", angleSpark.getAppliedOutput());

    SmartDashboard.putNumber("Coral Sensor", frontCoralRange);
  }

  public void runWristClosedLoop() {
      angleSetpoint = angleTrapezoidProfile.calculate(Constants.kDt, angleSetpoint, angleGoal);
		  angleController.setReference(angleSetpoint.position, ControlType.kPosition);
  }

  //----------//
  // Commands //
  //----------//

}
