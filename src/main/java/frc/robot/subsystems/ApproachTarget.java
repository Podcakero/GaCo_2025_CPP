// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

/** Reef target positions using blue side AprilTag IDs */
public enum ApproachTarget {
    REEF_A(18, true),
    REEF_B(18, false),
    REEF_AB(18, false),
    REEF_C(17, true),
    REEF_D(17, false),
    REEF_CD(17, false),
    REEF_E(22, true),
    REEF_F(22, false),
    REEF_EF(22, false),
    REEF_G(21, true),
    REEF_H(21, false),
    REEF_GH(21, false),
    REEF_I(20, true),
    REEF_J(20, false),
    REEF_IJ(20, false),
    REEF_K(19, true),
    REEF_L(19, false),
    REEF_KL(19, false);

    public final int tagId;
    public final boolean isLeft;

    ApproachTarget(int tagId, boolean isLeft){
        this.tagId = tagId;
        this.isLeft = isLeft;
    }
}
