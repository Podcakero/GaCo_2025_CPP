// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

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

        public static final double elevatorHomeHeightInches = 17.5;  // only valid when elevator is homed;

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

        public static final double kHeightTolleranceInches = 1.0;

        public static final int kElevatorCurrentLimit = 60;

        public static final int kElevatorMotorLeftId = 51;
        public static final int kElevatorMotorCenterId = 52;
        public static final int kElevatorMotorRightId = 53;

        public static final double kElevatorMaxHeightInches = 73;
        public static final double kElevatorSpeedSafeHeightInches = 40;
        public static final double kElevatorMinHeightInches = 17.5;

        public static final double kIntakeHeightInches = 17.5; 

        public static final double kL1CoralHeightInches = 21;
        public static final double kL2CoralHeightInches = 31;
        public static final double kL3CoralHeightInches = 46;
        public static final double kL4CoralHeightInches = 70;  // was 70

        public static final double kL1AlgaeHeightInches = 24;
        public static final double kL2AlgaeHeightInches = 39;
        public static final double kL3AlgaeHeightInches = 54;  

        public static final double kL4AlgaeWindupHeightInches = 70.5;  

        public static final double kSafeHomeHeightInches = 19;
        
        public static final double kElevatorMaxVelocityMPS = 2.0;  // MPS
		public static final double kElevatorMaxAccelerationMPSPS = 4.0; // MPSS  was 6
	
        public static final double kElevatorEncoderPositionConversionFactor = kRelativeEncoderScaleRevToMeters; 
        public static final double kElevatorEncoderVelocityConversionFactor = kRelativeEncoderScaleRevToMeters; 
    }

    public class DriverConstants{
        
        // driver
        public static final double kMaxDriveSpeed = 0.9;
        public static final double kMaxTurnSpeed  = 2;

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

    public class ApproachConstants {
		public static final double maxApproachLinearVelocityMPS = 2.0;
		public static final double maxApproachLinearAccelerationMPSPS = 1.5;
		public static final double maxApproachAngularVelocityRPS = 2 * Math.PI;
		public static final double maxApproachAngularAccelerationRPSPS = 4 * Math.PI;
	}
}
