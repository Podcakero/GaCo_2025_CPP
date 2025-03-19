// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

/** Reef target positions using blue side AprilTag IDs */
public enum ApproachTarget {
    UNKNOWN(0, ApproachPosition.ALGAE, false),
    REEF_A(18, ApproachPosition.LEFT, true),
    REEF_B(18, ApproachPosition.RIGHT, true),
    REEF_AB(18, ApproachPosition.ALGAE, true),
    REEF_C(17, ApproachPosition.LEFT, true),
    REEF_D(17, ApproachPosition.RIGHT, true),
    REEF_CD(17, ApproachPosition.ALGAE, true),
    REEF_E(22, ApproachPosition.LEFT, true),
    REEF_F(22, ApproachPosition.RIGHT, true),
    REEF_EF(22, ApproachPosition.ALGAE, true),
    REEF_G(21, ApproachPosition.LEFT, true),
    REEF_H(21, ApproachPosition.RIGHT, true),
    REEF_GH(21, ApproachPosition.ALGAE, true),
    REEF_I(20, ApproachPosition.LEFT, true),
    REEF_J(20, ApproachPosition.RIGHT, true),
    REEF_IJ(20, ApproachPosition.ALGAE, true),
    REEF_K(19, ApproachPosition.LEFT, true),
    REEF_L(19, ApproachPosition.RIGHT, true),
    REEF_KL(19, ApproachPosition.ALGAE, true),
    PROCESSOR(16,ApproachPosition.ALGAE, true),
    BARGE(14, ApproachPosition.OVERHEAD, false),
    LEFT_SOURCE(13, ApproachPosition.ALGAE, false),
    RIGHT_SOURCE(12, ApproachPosition.ALGAE, false)
    ;


    public ApproachPosition position;
    public final int tagId;
    public final boolean highCamLockout;

    ApproachTarget(int tagId, ApproachPosition position, boolean lockout){
        this.tagId = tagId;
        this.position = position;
        this.highCamLockout = lockout;
    }
}
