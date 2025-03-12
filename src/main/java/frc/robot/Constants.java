// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;

/** Add your docs here. */
public class Constants {

    public static final double kDt = 0.02;

    public class WristConstants {
        public static final int kAngleMotorId = 61;
        public static final int kIntakeMotorId = 62;
        public static final int kExitTOFId = 63;
        public static final int kEnterTOFId = 64;
        
        public static final double kAngleFactor = 360 * 24 / 40; // 216 degrees

        public static final double kP = 0.02;
        public static final double kI = 0;
        public static final double kD = 0.0;
        public static final double kAngleTollerance = 1;
		
        public static final double kAnglePower = 1;
		
		public static final double kCoralIntakePower = -0.3;
        public static final double kCoralSlowIntakePower = -0.05;
        public static final double kCoralRetractPower = 0.02;
		public static final double kCoralOutputPower =  0.5;
		public static final double kCoralScoringPower = -1.0;
		public static final double kAlgaeIntakePower = 0.3;
        public static final double kLowAlgaeIntakePower = 0.2;
    

        public static final double kAngleMaxVelocityDPS = 400; // 
		public static final double kAngleMaxAccelerationDPSPS = 1000; // 

        public static final double kIntakeAngle             = 4;
        public static final double kSafeAngle               = 30;
        public static final double kL4Angle                 = 48;
        public static final double kHighAlgaeAngle          = 60;

        public static final double kAlgaeIntakeAngle        = 180;  // was 155
        
        public static final double kMaxAngleWhenHome        = 182;
        public static final double kMaxAngle                = 210;

        public static final double kMaxCoralDetectRangeMM   = 80;
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

        public static final Distance kElevatorMaxHeight = Inches.of(73);
        public static final Distance kElevatorSpeedSafeHeight = Inches.of(40);
        public static final Distance kElevatorMinHeight = Inches.of(17.5);

        public static final Distance kIntakeHeight = Inches.of(17.5); 

        public static final Distance kL1CoralHeight = Inches.of(21);
        public static final Distance kL2CoralHeight = Inches.of(31);
        public static final Distance kL3CoralHeight = Inches.of(46);
        public static final Distance kL4CoralHeight = Inches.of(72);

        public static final Distance kL1AlgaeHeight = Inches.of(24);
        public static final Distance kL2AlgaeHeight = Inches.of(38);
        public static final Distance kL3AlgaeHeight = Inches.of(54);  // was 47.5

        public static final Distance kSafeHomeHeight = Inches.of(19);
        

        public static final double kElevatorMaxVelocityRPS = 2.0;  // MPS
		public static final double kElevatorMaxAccelerationRPSPS = 4.0; // MPSS  was 6
	
        public static final double kElevatorEncoderPositionConversionFactor = kRelativeEncoderScaleRevToMeters; // Needs to be empirically measured
        public static final double kElevatorEncoderVelocityConversionFactor = kRelativeEncoderScaleRevToMeters; // Needs to be empirically measured. Should be able to be derived from kElevatorEncoderPositionConversionFactor
   
    }

    public class DriverConstants{
        
        // driver
        public static final double kMaxDriveSpeed = 0.85;
        public static final double kMaxTurnSpeed  = 0.9;
        

        //Co-Pilot 1
        
        public static final int reset = 1;

        public static final int l4 = 2;
        public static final int l3 = 3;
        public static final int l2 = 4;
        public static final int l1 = 5;

        public static final int unknown = 6;

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
}
