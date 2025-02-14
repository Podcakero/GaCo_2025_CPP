// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Constants {
    public class WristConstants {
        public static final int kWristMotorId = 3;
    }

    public class ElevatorConstant {
        public static final double kP = 1.0;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final int kElevatorCurrentLimit = 40;

        public static final int kElevatorEncoderId = 50;
        public static final int kElevatorMotorLeftId = 51;
        public static final int kElevatorMotorCenterId = 52;
        public static final int kElevatorMotorRightId = 53;
    }
}
