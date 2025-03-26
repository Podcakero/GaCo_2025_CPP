// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;

/** Add your docs here. */
public class Constants {

    public static final double kDt = 0.02;

    public static final AprilTagFields kField = AprilTagFields.k2025ReefscapeAndyMark;
    public static final AprilTagFieldLayout kFieldLayout = AprilTagFieldLayout.loadField(kField);

    public class Wrist {
        public static final int kAngleMotorId = 61;
        public static final int kIntakeMotorId = 62;
        public static final int kExitTOFId = 63;
        public static final int kEnterTOFId = 64;

		public static final int kAlgaeGrabbedCurrent = 40;
        
        public static final double kAngleFactor = 360 * 24 / 40; // 216 degrees

        public static final double kP = 0.04; // was 0.02
        public static final double kI = 0;
        public static final double kD = 0.0;
        public static final double kAngleTollerance = 1;
		
        public static final double kAnglePower = 0.5; // was 1.0
		
		public static final double kCoralIntakePower = -0.3;
        public static final double kCoralSlowIntakePower = -0.1;
        public static final double kCoralRetractPower = 0.02;
		public static final double kCoralOutputPower =  0.5;
		public static final double kCoralL234ScoringPower = -1.0;
        public static final double kCoralL1ScoringPower = -0.2;

		public static final double kAlgaeIntakePower = 0.3;
        public static final double kLowAlgaeIntakePower = 0.2;

        public static final double kAngleMaxVelocityDPS = 400; // 
		public static final double kAngleMaxAccelerationDPSPS = 1000; // 

        public static final double kIntakeAngleDegrees             = 2;
        public static final double kSafeAngleDegrees               = 30;
        public static final double kL4AngleDegrees                 = 48;
        public static final double kHighAlgaeAngleDegrees          = 60;

        public static final double kAlgaeIntakeAngleDegrees        = 180; 

        public static final double kAlgaeWindupAngleDegrees        = 150; 
        public static final double kAlgaeReleaseAngleDegrees       = 60; 
        
        public static final double kMaxAngleWhenHomeDegrees        = 182;
        public static final double kMaxAngleDegrees                = 210;

        public static final double kMaxCoralDetectRangeMM   = 80;

        public static final double kTOFSampleTime = 24;
    }

    public class Elevator {

        public static final double elevatorHomeHeightMeters = Units.inchesToMeters(17.5);  // only valid when elevator is homed;

        // scale factors
        public static final double kRelativeEncoderScaleRevToMeters = 0.0315;  // 25 turns for 31 inches of travel
        public static final double kAbsoluteEncoderScaleVoltsToMeters = 0.498;
        public static final double kAbsoluteEncoderOffsetVoltsToMeters = 0.43;

        public static final double kP = 6;  // was 8
        public static final double kI = 0;
        public static final double kD = 0.46;

        public static final double kS = 0.04435;    //  these may need to be rescaled for meters by dividing 
		public static final double kG = 0.257;      //  by kRelativeEncoderScaleRevToMeters
		public static final double kV = 0.117 ;     //
		public static final double kA = 0.00803 ;   //

        public static final double kHeightTolleranceMeters = Units.inchesToMeters(1.0);

        public static final int kElevatorCurrentLimit = 60;

        public static final int kElevatorMotorLeftId = 51;
        public static final int kElevatorMotorCenterId = 52;
        public static final int kElevatorMotorRightId = 53;

        public static final double kElevatorMaxHeightMeters = Units.inchesToMeters(73);
        public static final double kElevatorSpeedSafeHeightMeters = Units.inchesToMeters(40);
        public static final double kElevatorMinHeightMeters = Units.inchesToMeters(17.5);

        public static final double kIntakeHeightMeters = Units.inchesToMeters(17.5); 

        public static final double kL1CoralHeightMeters = Units.inchesToMeters(21);
        public static final double kL2CoralHeightMeters = Units.inchesToMeters(31);
        public static final double kL3CoralHeightMeters = Units.inchesToMeters(46);
        public static final double kL4CoralHeightMeters = Units.inchesToMeters(70);  // was 70

        public static final double kL1AlgaeHeightMeters = Units.inchesToMeters(24);
        public static final double kL2AlgaeHeightMeters = Units.inchesToMeters(39);
        public static final double kL3AlgaeHeightMeters = Units.inchesToMeters(54);  

        public static final double kL4AlgaeWindupHeightMeters = Units.inchesToMeters(70.5);  

        public static final double kSafeHomeHeightMeters = Units.inchesToMeters(19);
        
        //public static final double kElevatorMaxVelocityMPS = 2.0;  // MPS
		public static final double kElevatorMaxAccelerationMPSPS = 4.0; // MPSS  was 6
	
        public static final double kElevatorEncoderPositionConversionFactor = kRelativeEncoderScaleRevToMeters; 
        public static final double kElevatorEncoderVelocityConversionFactor = kRelativeEncoderScaleRevToMeters; 
    }

    public class DriverConstants{
        
        // driver 
        public static final double kMaxDriveSpeed = 1.0; // 100% of RobotContainer.MaxSpeed
        public static final double kMaxTurnSpeed  = 1.0; // 100% of RobotContainer.MaxAngularRate

        //Co-Pilot 1
        public static final int reset = 1;

        public static final int l4 = 2;
        public static final int l3 = 3;
        public static final int l2 = 4;
        public static final int l1 = 5;

        public static final int home = 6;

        public static final int pose_ija = 7;
        public static final int pose_i = 8;
        public static final int pose_j = 9;
        public static final int pose_kla = 10;
        public static final int pose_k = 11;
        public static final int pose_l = 12;

        //Co-pilot 2
        public static final int pose_h = 1;
        public static final int pose_gha = 2;
        public static final int pose_g = 3;
        public static final int pose_f = 4;
        public static final int pose_efa = 5;
        public static final int pose_e = 6;
        public static final int pose_cda = 7;
        public static final int pose_d = 8;
        public static final int pose_c = 9;
        public static final int pose_b = 10;
        public static final int pose_aba = 11;
        public static final int pose_a = 12;

    }

    public class DrivetrainConstants {
        public static final double kMaxVelocityMPS = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        public static final double kMaxAccelerationMPSPS = 2 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        public static final double kMaxAngularVelocityRPS = 2 * Math.PI;
        public static final double kMaxAngularAccelerationRPSPS = 4 * Math.PI;

        public static final double kPHeading = 0.0;
        public static final double kIHeading = 0.0;
        public static final double kDHeading = 0.0;
    }

    public class ApproachConstants {
		public static final double maxApproachLinearVelocityPercent = 0.5; // Was 2.0m/s, now percent of kMaxVelocityMPS
		public static final double maxApproachLinearAccelerationPercent = 0.5; // Was 1.5m/s, now percent of kMaxAccelerationMPSPS
		public static final double maxApproachAngularVelocityPercent = 0.5; // Was 2PI, now percent of kMaxAngularVelocityRPS
		public static final double maxApproachAngularAccelerationPercent = 0.5; // Was 4PI, now percent of kMaxAngularAccelerationRPSPS
	}
}
