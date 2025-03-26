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

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
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
import frc.robot.commands.DefaultWristCmd;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.Utils;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

public class WristSubsystem extends SubsystemBase {

  private final SparkFlex intakeSpark;
  private SparkFlex angleSpark; // cannot be final due to SysID
  private final SparkFlexConfig intakeConfig = new SparkFlexConfig();
  private final SparkFlexConfig angleConfig = new SparkFlexConfig();
  private final SparkFlexConfig resetFrameRateConfig = new SparkFlexConfig();

  private final RelativeEncoder intakeEncoder;
  private final AbsoluteEncoder angleEncoder;

  private final SparkClosedLoopController angleController;
  private final ArmFeedforward angleFeedforward;
  private final TrapezoidProfile angleTrapezoidProfile;

	private TrapezoidProfile.State angleGoal = new TrapezoidProfile.State();
	private TrapezoidProfile.State angleSetpoint;

  private final TimeOfFlight enterTOF;
  private final TimeOfFlight exitTOF;

  private double exitCoralRangeMM = 0;
  private double enterCoralRangeMM = 0;

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutAngle m_distance = Degrees.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutAngularVelocity m_velocity = DegreesPerSecond.mutable(0);

  private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
    new SysIdRoutine.Config(
      null,
      Volts.of(1),
      null,
      null
    ),
    new SysIdRoutine.Mechanism(
      output -> setVoltage(output),
      log -> {
        log.motor("Angle")
            .voltage(
                m_appliedVoltage.mut_replace(
                    angleSpark.getAppliedOutput() * RobotController.getBatteryVoltage(), Volts))
            .angularPosition(m_distance.mut_replace(angleSpark.getEncoder().getPosition(), Degrees))
            .angularVelocity(
                m_velocity.mut_replace(angleSpark.getEncoder().getVelocity(), DegreesPerSecond));
      },
      this
    )
  );
  
  /** Creates a new WristSubsystem. */
  public WristSubsystem() {

    intakeSpark = new SparkFlex(Constants.Wrist.kIntakeMotorId, MotorType.kBrushless);
    angleSpark = new SparkFlex(Constants.Wrist.kAngleMotorId, MotorType.kBrushless);

    intakeEncoder = intakeSpark.getEncoder();
    angleEncoder = angleSpark.getAbsoluteEncoder();

    angleFeedforward = new ArmFeedforward(Constants.Wrist.kS, Constants.Wrist.kG, Constants.Wrist.kG, Constants.Wrist.kA);

    angleController = angleSpark.getClosedLoopController();

    // Use module constants to calculate conversion factors and feed forward gain.
    double intakeFactor = 1;
    // Sprocket reduction

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.

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
    exitTOF.setRangingMode(RangingMode.Short, Constants.Wrist.kTOFSampleTime);
    exitTOF.setRangeOfInterest(0, 0, 15, 15);
    
    enterTOF = new TimeOfFlight(Constants.Wrist.kEnterTOFId);
    enterTOF.setRangingMode(RangingMode.Short, Constants.Wrist.kTOFSampleTime);
    enterTOF.setRangeOfInterest(0, 0, 15, 15);

    resetFrameRateConfig.signals.appliedOutputPeriodMs(10);

    setDefaultCommand(new DefaultWristCmd(this));

    angleTrapezoidProfile = new TrapezoidProfile(new Constraints(
                                                      angleFeedforward.maxAchievableVelocity(
                                                          12.0, 
                                                          angleEncoder.getPosition(), 
                                                          Constants.Wrist.kAngleMaxAccelerationDPSPS),
				                                              Constants.Wrist.kAngleMaxAccelerationDPSPS));

	  angleSetpoint = new TrapezoidProfile.State(angleEncoder.getPosition(), angleEncoder.getVelocity());
  }

  public void initialize() {
    setIntakeSpeed(0);
  }

  public void resetFrameRate() {
    intakeSpark.configure(resetFrameRateConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    angleSpark.configure(resetFrameRateConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  // The configuration interfaces may be accessed by typing in the IP address of the roboRIO into a web
  //  browser followed by :5812.
  @Override
  public void simulationPeriodic() {
      // TODO Auto-generated method stub
      SmartDashboard.putNumber("Wrist Goal", angleGoal.position);    
      SmartDashboard.putNumber("Wrist Angle", angleGoal.position);

      SmartDashboard.putString("Wrist Power", "SIMULATION");
      SmartDashboard.putNumber("Exit Coral Sensor", exitCoralRangeMM);
      SmartDashboard.putNumber("Enter Coral Sensor", enterCoralRangeMM);
  }

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
    SmartDashboard.putNumber("Exit Coral Sensor", exitCoralRangeMM);
    SmartDashboard.putNumber("Enter Coral Sensor", enterCoralRangeMM);
    
  }

  // TOF sensor
  public void setViewZone(int topLeftX, int topLeftY, int bottomRightX, int bottomRightY) {
    exitTOF.setRangeOfInterest(topLeftX, topLeftY, bottomRightX, bottomRightY);
    enterTOF.setRangeOfInterest(topLeftX, topLeftY, bottomRightX, bottomRightY);
  }

  public boolean gotExitCoral() {
    if (Utils.isSimulation()){
      return true;
    } else {
      return (exitCoralRangeMM < Constants.Wrist.kMaxCoralDetectRangeMM);
    }
  }

  public boolean gotEnterCoral() {
    if (Utils.isSimulation()){
      return true;
    } else {
      return (exitCoralRangeMM < Constants.Wrist.kMaxCoralDetectRangeMM);
    }
  }

  public void getRangeMM() {
    exitCoralRangeMM = exitTOF.getRange();
    enterCoralRangeMM = enterTOF.getRange();
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
    angleGoal.position = angle;
    angleGoal.velocity = 0.0;
    
    angleSetpoint.position = getWristAngle();
    angleSetpoint.velocity = 0.0;
  }

  public double getWristAngle(){
    return angleEncoder.getPosition();
  }

  public double getWristSpeed(){
    return angleEncoder.getVelocity();
  }


  public boolean inPosition(){
    if (Utils.isSimulation()){
      Globals.WRIST_IN_POSITION = true;
      return Globals.WRIST_IN_POSITION;
    } else {
      Globals.WRIST_IN_POSITION = (Math.abs(angleGoal.position - getWristAngle()) < Constants.Wrist.kAngleTollerance);
      return Globals.WRIST_IN_POSITION;
    }
  }

  
  public void runWristClosedLoop() {
      angleSetpoint = angleTrapezoidProfile.calculate(Constants.kDt, angleSetpoint, angleGoal);
		  angleController.setReference(angleSetpoint.position, ControlType.kPosition);
  }

  private void setVoltage(Voltage voltage) {
    angleSpark.setVoltage(voltage);
  }

  public Command sysIdDynamic(Direction direction)
  {
    return m_sysIdRoutine.dynamic(direction);
  }

  public Command sysIdQuasistatic(Direction direction)
  {
    return m_sysIdRoutine.quasistatic(direction);
  }

  //----------//
  // Commands //
  //----------//

}
