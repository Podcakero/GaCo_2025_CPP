// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

/** Reef target positions using blue side AprilTag IDs */
public enum ApproachTarget {
    UNKNOWN(0, ApproachPosition.ALGAE),
    REEF_A(18, ApproachPosition.LEFT),
    REEF_B(18, ApproachPosition.RIGHT),
    REEF_AB(18, ApproachPosition.ALGAE),
    REEF_C(17, ApproachPosition.LEFT),
    REEF_D(17, ApproachPosition.RIGHT),
    REEF_CD(17, ApproachPosition.ALGAE),
    REEF_E(22, ApproachPosition.LEFT),
    REEF_F(22, ApproachPosition.RIGHT),
    REEF_EF(22, ApproachPosition.ALGAE),
    REEF_G(21, ApproachPosition.LEFT),
    REEF_H(21, ApproachPosition.RIGHT),
    REEF_GH(21, ApproachPosition.ALGAE),
    REEF_I(20, ApproachPosition.LEFT),
    REEF_J(20, ApproachPosition.RIGHT),
    REEF_IJ(20, ApproachPosition.ALGAE),
    REEF_K(19, ApproachPosition.LEFT),
    REEF_L(19, ApproachPosition.RIGHT),
    REEF_KL(19, ApproachPosition.ALGAE),
    PROCESSOR(16,ApproachPosition.ALGAE),
    BARGE(14, ApproachPosition.OVERHEAD),
    LEFT_SOURCE(13, ApproachPosition.ALGAE),
    RIGHT_SOURCE(12, ApproachPosition.ALGAE)
    ;


    public ApproachPosition position;
    public final int tagId;

    ApproachTarget(int tagId, ApproachPosition position){
        this.tagId = tagId;
        this.position = position;
    }
}
