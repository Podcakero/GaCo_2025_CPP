// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.pathplanner.lib.path.GoalEndState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

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
    RIGHT_SOURCE(12, ApproachPosition.ALGAE, true);

    public final int tagId;
    public final boolean enableHighCam;

    public final ApproachPosition position;

    public final Pose2d tagPose;
    public final Pose2d pt1;
    public final Pose2d pt2;

    public final GoalEndState goalEndState;

    private ApproachTarget(int tagId, ApproachPosition position, boolean enableHighCam){
        this.tagId = getTagId(tagId);
        this.enableHighCam = enableHighCam;

        this.position = position;

        this.tagPose = (tagId == 0) ? Pose2d.kZero : Constants.kFieldLayout.getTagPose(tagId).get().toPose2d();
        this.pt1 = tagPose.plus(position.pt1Transform);
        this.pt2 = tagPose.plus(position.pt2Transform);

        this.goalEndState = new GoalEndState(0.0, (position == ApproachPosition.OVERHEAD) ? tagPose.getRotation() : tagPose.getRotation().rotateBy(Rotation2d.k180deg));
    }

    // Modify tag ID is running on Red Alliance.
    public static int getTagId(int id){
        if(DriverStation.getAlliance() != null && DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(Alliance.Red)){
            switch(id){
                case 12:
                    return 2;
                case 13:
                    return 1;
                case 14:
                    return 5;
                case 16:
                    return 3;
                case 17:
                    return 8;
                case 18:
                    return 7;
                case 19:
                    return 6;
                case 20:
                    return 11;
                case 21:
                    return 10;
                case 22:
                    return 9;
            }
        }
        return id;
    }
}
