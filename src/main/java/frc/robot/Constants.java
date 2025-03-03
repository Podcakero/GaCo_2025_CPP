// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;

/** Add your docs here. */
public class Constants {

    public static final double kDt = 0.02;

    public class WristConstants {
        public static final int kAngleMotorId = 61;
        public static final int kIntakeMotorId = 62;

        public static final double kP = 0.02;
        public static final double kI = 0;
        public static final double kD = 0.0;
        public static final double kAngleTollerance = 1;
		
        public static final double kAnglePower = 1;
		
		public static final double kCoralIntakePower = -0.2;
		public static final double kCoralOutputPower =  0.5;
		public static final double kCoralScoringPower = -1.0;

        public static final double kAngleMaxVelocityDPS = 400; // 
		public static final double kAngleMaxAccelerationDPSPS = 1000; // 

        public static final double kMaxCoralDetectRangeMM = 40;
        public static final double kSafeAngle = 30;
        public static final double kIntakeAngle = 3;
        public static final double kL4Angle = 48;

    }

    public class ElevatorConstants {

        // scale factors
        public static final double kRelativeEncoderScaleRevToMeters = 0.0315;  // 25 turns for 31 inches of travel
        public static final double kAbsoluteEncoderScaleVoltsToMeters = 0.498;
        public static final double kAbsoluteEncoderOffsetVoltsToMeters = 0.43;

        public static final double kP = 6;  // was 8
        public static final double kI = 0;
        public static final double kD = 0.46;

        public static final double kS = 0.04435;    //  these may need to be rescaled for meters by dividing 
		public static final double kG = 0.257;      //  by kRelativeEncoderScaleRevToMeters
		public static final double kV = 0.117 ;      //
		public static final double kA = 0.00803 ;    //

        public static final Distance kHeightTollerance = Inches.of(1.0);

        public static final int kElevatorCurrentLimit = 60;

        public static final int kElevatorMotorLeftId = 51;
        public static final int kElevatorMotorCenterId = 52;
        public static final int kElevatorMotorRightId = 53;

        public static final Distance kL1Height = Inches.of(21);
        public static final Distance kL2Height = Inches.of(24);
        public static final Distance kL3Height = Inches.of(28);
        public static final Distance kL4Height = Inches.of(36); // was 2.1m
        public static final Distance kIntakeHeight = Meters.of(18); 

        public static final double kElevatorMaxVelocityRPS = 2.0;  // MPS
		public static final double kElevatorMaxAccelerationRPSPS = 6.0; // MPSS
	
        public static final double kElevatorEncoderPositionConversionFactor = kRelativeEncoderScaleRevToMeters; // Needs to be empirically measured
        public static final double kElevatorEncoderVelocityConversionFactor = kRelativeEncoderScaleRevToMeters; // Needs to be empirically measured. Should be able to be derived from kElevatorEncoderPositionConversionFactor

        
    }
}
