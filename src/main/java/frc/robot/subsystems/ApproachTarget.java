// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

/** Reef target positions using blue side AprilTag IDs */
public enum ApproachTarget {
    UNKNOWN(0, ReefSidePosition.CENTER),
    REEF_A(18, ReefSidePosition.LEFT),
    REEF_B(18, ReefSidePosition.RIGHT),
    REEF_AB(18, ReefSidePosition.CENTER),
    REEF_C(17, ReefSidePosition.LEFT),
    REEF_D(17, ReefSidePosition.RIGHT),
    REEF_CD(17, ReefSidePosition.CENTER),
    REEF_E(22, ReefSidePosition.LEFT),
    REEF_F(22, ReefSidePosition.RIGHT),
    REEF_EF(22, ReefSidePosition.CENTER),
    REEF_G(21, ReefSidePosition.LEFT),
    REEF_H(21, ReefSidePosition.RIGHT),
    REEF_GH(21, ReefSidePosition.CENTER),
    REEF_I(20, ReefSidePosition.LEFT),
    REEF_J(20, ReefSidePosition.RIGHT),
    REEF_IJ(20, ReefSidePosition.CENTER),
    REEF_K(19, ReefSidePosition.LEFT),
    REEF_L(19, ReefSidePosition.RIGHT),
    REEF_KL(19, ReefSidePosition.CENTER),
    PROCESSOR(16,ReefSidePosition.CENTER),
    BARGE(14, ReefSidePosition.CENTER),
    LEFT_SOURCE(13, ReefSidePosition.CENTER),
    RIGHT_SOURCE(12, ReefSidePosition.CENTER)
    ;


    public ReefSidePosition position;
    public final int tagId;

    ApproachTarget(int tagId, ReefSidePosition position){
        this.tagId = tagId;
        this.position = position;
    }
}
