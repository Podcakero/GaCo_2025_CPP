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

  TimeOfFlight enterTOF;
  TimeOfFlight exitTOF;
  double exitCoralRange = 0;
  double enterCoralRange = 0;
  
  /** Creates a new WristSubsystem. */
  public WristSubsystem() {

    intakeSpark = new SparkFlex(Constants.Wrist.kIntakeMotorId, MotorType.kBrushless);
    angleSpark = new SparkFlex(Constants.Wrist.kAngleMotorId, MotorType.kBrushless);

    intakeEncoder = intakeSpark.getEncoder();
    angleEncoder = angleSpark.getAbsoluteEncoder();

    angleTrapezoidProfile = new TrapezoidProfile(new Constraints(Constants.Wrist.kAngleMaxVelocityDPS,
				                                              Constants.Wrist.kAngleMaxAccelerationDPSPS));

	  angleSetpoint = new TrapezoidProfile.State(angleEncoder.getPosition(), angleEncoder.getVelocity());
    angleController = angleSpark.getClosedLoopController();

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.

    SparkFlexConfig intakeConfig = new SparkFlexConfig();
    SparkFlexConfig angleConfig = new SparkFlexConfig();

    // Use module constants to calculate conversion factors and feed forward gain.
    double intakeFactor = 1;
     // Sprocket reduction


    intakeConfig
      .idleMode(IdleMode.kBrake)
      .smartCurrentLimit(50);
    intakeConfig.encoder
      .positionConversionFactor(intakeFactor) // meters
      .velocityConversionFactor(intakeFactor / 60.0); // meters per second
    
    angleConfig
      .idleMode(IdleMode.kBrake)
      //.inverted(true)
      .smartCurrentLimit(50);
    angleConfig.absoluteEncoder
      //.inverted(true)
      .positionConversionFactor(Constants.Wrist.kAngleFactor) // degrees
      .velocityConversionFactor(Constants.Wrist.kAngleFactor / 60.0); // degrees per second
    angleConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      // These are example gains you may need to them for your own robot!
      .pid(Constants.Wrist.kP, Constants.Wrist.kI, Constants.Wrist.kD)
      .outputRange(-Constants.Wrist.kAnglePower, Constants.Wrist.kAnglePower)
      .positionWrappingInputRange(0, Constants.Wrist.kAngleFactor);

    intakeSpark.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    angleSpark.configure(angleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    exitTOF = new TimeOfFlight(Constants.Wrist.kExitTOFId);
    exitTOF.setRangingMode(RangingMode.Short, 30);
    exitTOF.setRangeOfInterest(0, 0, 15, 15);
    
    enterTOF = new TimeOfFlight(Constants.Wrist.kEnterTOFId);
    enterTOF.setRangingMode(RangingMode.Short, 30);
    enterTOF.setRangeOfInterest(0, 0, 15, 15);

    setDefaultCommand( new DefaultWristCmd(this));
  }

  public void initialize() {
    setIntakeSpeed(0);
  }

  public void resetFrameRate() {
    SparkFlexConfig config = new SparkFlexConfig();
    config.signals.appliedOutputPeriodMs(10);
    
    intakeSpark.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    angleSpark.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  // The configuration interfaces may be accessed by typing in the IP address of the roboRIO into a web
  //  browser followed by :5812.
  @Override
  public void periodic() {

    getRangeMM();

    Globals.GOT_CORAL = gotExitCoral() && gotEnterCoral();

     /*if (DriverStation.getStickButtonPressed(1,2)){
      bumpWrist(0.1016);
    } else if (DriverStation.getStickButtonPressed(1,3)){
      bumpWrist(0.0254);
    } else if (DriverStation.getStickButtonPressed(1,4)){
      bumpWrist(-0.0254);
    } else if (DriverStation.get(1,5)){
      bumpWrist(-0.1016);
    }*/

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Wrist Goal", angleGoal.position);    
    SmartDashboard.putNumber("Wrist Angle", getWristAngle());

    SmartDashboard.putNumber("Wrist Power", angleSpark.getAppliedOutput());
    SmartDashboard.putNumber("Exit Coral Sensor", exitCoralRange);
    SmartDashboard.putNumber("Enter Coral Sensor", enterCoralRange);
    
  }

  // TOF sensor
  public void setViewZone(int topLeftX, int topLeftY, int bottomRightX, int bottomRightY) {
    exitTOF.setRangeOfInterest(topLeftX, topLeftY, bottomRightX, bottomRightY);
    enterTOF.setRangeOfInterest(topLeftX, topLeftY, bottomRightX, bottomRightY);
  }

  public boolean gotExitCoral() {
    return (exitCoralRange < Constants.Wrist.kMaxCoralDetectRangeMM);
  }

  public boolean gotEnterCoral() {
    return (enterCoralRange < Constants.Wrist.kMaxCoralDetectRangeMM);
  }

  public void getRangeMM() {
    exitCoralRange = exitTOF.getRange();
    enterCoralRange = enterTOF.getRange();
  }
      
  // intake
  public void setIntakeSpeed(double speed) {
    intakeSpark.set(speed);
  }

  public double getIntakeSpeed() {
    return intakeEncoder.getVelocity();
  }

  public double getIntakeCurrent() {
    return intakeSpark.getOutputCurrent();
  }

  // wrist
  public void resetWristControl () {
    setIntakeSpeed(0);
    setGoalAngle(getWristAngle());
  }

  public void setGoalAngle(double angle){
    angleGoal = new TrapezoidProfile.State(angle, 0.0);
    angleSetpoint = new TrapezoidProfile.State(getWristAngle(), 0.0);
  }

  public double getWristAngle(){
    return angleEncoder.getPosition();
  }

  public double getWristSpeed(){
    return angleEncoder.getVelocity();
  }


  public boolean inPosition(){
    Globals.WRIST_IN_POSITION = (Math.abs(angleGoal.position - getWristAngle()) < Constants.Wrist.kAngleTollerance);
    return Globals.WRIST_IN_POSITION;
  }

  
  public void runWristClosedLoop() {
      angleSetpoint = angleTrapezoidProfile.calculate(Constants.kDt, angleSetpoint, angleGoal);
		  angleController.setReference(angleSetpoint.position, ControlType.kPosition);
  }



  //----------//
  // Commands //
  //----------//

}
