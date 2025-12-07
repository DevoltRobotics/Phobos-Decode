package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
@Config
public class DrivePos {

    ///  RED
    /// CLOSE
    public static Pose startingPoseCloseRed = new Pose(124, 122, Math.toRadians(40));

    public static Pose launchPreloadCloseRedPose = new Pose(97, 97, Math.toRadians(40));

    public static Pose prepareForIntakeFirstCloseRedPose = new Pose(100, 89, Math.toRadians(0));

    public static Pose intakeFirstCloseRedPose = new Pose(123, 89, Math.toRadians(0));

    public static Pose openGateCloseRedPose = new Pose(128, 72, Math.toRadians(270));
    public static Pose openGateControlPointCloseRedPose = new Pose(102, 79, Math.toRadians(0));

    public static Pose launchFirstCloseRedPose = new Pose(90, 90, Math.toRadians(330));

    public static Pose prepareForIntakeSecondCloseRedPose = new Pose(100, 67, Math.toRadians(350));

    public static Pose intakeSecondCloseRedPose = new Pose(126, 65, Math.toRadians(350));

    public static Pose launchSecondCloseRedPose = new Pose(90, 90, Math.toRadians(350));

    public static Pose prepareForIntakeThirdCloseRedPose = new Pose(100, 35, Math.toRadians(0));

    public static Pose intakeThirdCloseRedPose = new Pose(128, 35, Math.toRadians(0));

    public static Pose launchThirdCloseRedPose = new Pose(90, 90, Math.toRadians(330));

    public static Pose parkCloseRedPose = new Pose(104, 75, Math.toRadians(0));

    /// FURTHER

    public static Pose startingPoseFurtherRed = new Pose(88, 7, Math.toRadians(90));

    public static Pose prepareForIntakeArtifactsFurtherRedPose = new Pose(102, 35, Math.toRadians(0));

    public static Pose intakeArtifactsFurtherRedPose = new Pose(129, 35, Math.toRadians(0));

    public static Pose launchSecondArtifactsFurtherRedPose = new Pose(87, 13, Math.toRadians(30));

    public static Pose parkFurtherRedPose = new Pose(90, 28, Math.toRadians(90));

    ///  BLUE

    /// CLOSE

    public static Pose startingPoseCloseBlue = startingPoseCloseRed.mirror();

    public static Pose launchFirstArtifactsCloseBluePose = launchFirstCloseRedPose.mirror();

    public static Pose parkCloseBluePose = parkCloseRedPose.mirror();


    /// FURTHER

    public static Pose startingPoseFurtherBlue = startingPoseFurtherRed.mirror();

    public static Pose parkFurtherBluePose = parkFurtherRedPose.mirror();

}
