// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

/** Reef target positions using blue side AprilTag IDs */
public enum ApproachTarget {
    UNKNOWN(0, ApproachPosition.ALGAE, false),
    REEF_A(18, ApproachPosition.LEFT, false),
    REEF_B(18, ApproachPosition.RIGHT, false),
    REEF_AB(18, ApproachPosition.ALGAE, false),
    REEF_C(17, ApproachPosition.LEFT, false),
    REEF_D(17, ApproachPosition.RIGHT, false),
    REEF_CD(17, ApproachPosition.ALGAE, false),
    REEF_E(22, ApproachPosition.LEFT, false),
    REEF_F(22, ApproachPosition.RIGHT, false),
    REEF_EF(22, ApproachPosition.ALGAE, false),
    REEF_G(21, ApproachPosition.LEFT, false),
    REEF_H(21, ApproachPosition.RIGHT, false),
    REEF_GH(21, ApproachPosition.ALGAE, false),
    REEF_I(20, ApproachPosition.LEFT, false),
    REEF_J(20, ApproachPosition.RIGHT, false),
    REEF_IJ(20, ApproachPosition.ALGAE, false),
    REEF_K(19, ApproachPosition.LEFT, false),
    REEF_L(19, ApproachPosition.RIGHT, false),
    REEF_KL(19, ApproachPosition.ALGAE, false),
    PROCESSOR(16,ApproachPosition.ALGAE, false),
    BARGE(14, ApproachPosition.OVERHEAD, true),
    LEFT_SOURCE(13, ApproachPosition.ALGAE, true),
    RIGHT_SOURCE(12, ApproachPosition.ALGAE, true)
    ;


    public ApproachPosition position;
    public final int tagId;
    public final boolean enableHighCam;

    ApproachTarget(int tagId, ApproachPosition position, boolean enableHighCam){
        this.tagId = tagId;
        this.position = position;
        this.enableHighCam = enableHighCam;
    }
}
