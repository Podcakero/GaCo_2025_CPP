// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Constants {

    public static final double kDt = 0.02;

    public class WristConstants {
        public static final int kWristMotorId = 3;
    }

    public class ElevatorConstants {
        public static final double kP = 0.4;
        public static final double kI = 0;
        public static final double kD = 0.023;

        public static final double kS = 0.04435;
		public static final double kG = 0.257;
		public static final double kV = 0.117;
		public static final double kA = 0.00803;

        public static final int kElevatorCurrentLimit = 40;

        public static final int kElevatorMotorLeftId = 51;
        public static final int kElevatorMotorCenterId = 52;
        public static final int kElevatorMotorRightId = 53;

        public static final double kElevatorMaxVelocityRPS = 80; // RPS
		public static final double kElevatorMaxAccelerationRPSPS = 100; // RPSPS
	
        public static final double kElevatorEncoderPositionConversionFactor = 1.0; // Needs to be empirically measured
        public static final double kElevatorEncoderVelocityConversionFactor = 1.0; // Needs to be empirically measured. Should be able to be derived from kElevatorEncoderPositionConversionFactor
    }
}
