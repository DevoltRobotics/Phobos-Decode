package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
@Config
public class DrivePos {

    ///  RED
    /// CLOSE
    public static Pose startingPoseCloseRed = new Pose(109, 135, Math.toRadians(0));

    public static Pose launchFirstArtifactsCloseRedPose = new Pose(98, 97, Math.toRadians(45));

    public static Pose prepareForIntakeArtifactsCloseRedPose = new Pose(126, 98, Math.toRadians(245));
    public static Pose prepareForIntakeArtifactsControlPointCloseRedPose = new Pose(98, 97, Math.toRadians(245));

    public static Pose intakeArtifactsCloseRedPose = new Pose(116, 83, Math.toRadians(180));
    public static Pose intakeArtifactsControlPointCloseRedPose = new Pose(133, 87, Math.toRadians(180));

    public static Pose launchSecondArtifactsCloseRedPose = new Pose(94, 91, Math.toRadians(300));

    public static Pose parkCloseRedPose = new Pose(100, 80, Math.toRadians(45));

    /// FURTHER

    public static Pose startingPoseFurtherRed = new Pose(88, 7, Math.toRadians(90));

    public static Pose prepareForIntakeArtifactsFurtherRedPose = new Pose(102, 35, Math.toRadians(0));

    public static Pose intakeArtifactsFurtherRedPose = new Pose(129, 35, Math.toRadians(0));

    public static Pose launchSecondArtifactsFurtherRedPose = new Pose(87, 13, Math.toRadians(30));

    public static Pose parkFurtherRedPose = new Pose(92, 32, Math.toRadians(30));

    ///  BLUE

    /// CLOSE

    public static Pose startingPoseCloseBlue = startingPoseCloseRed.mirror();

    /// FURTHER


}
