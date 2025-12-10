package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
@Config
public class DrivePos {

    ///  RED
    ///
    /// CLOSE
    public static Pose startingPoseCloseRed = new Pose(124, 122, Math.toRadians(40));

    public static Pose launchPreloadCloseRedPose = new Pose(93, 93, Math.toRadians(40));

    public static Pose prepareForIntakeFirstCloseRedPose = new Pose(100, 84, Math.toRadians(0));

    public static Pose intakeFirstCloseRedPose = new Pose(118, 84, Math.toRadians(0));

    public static Pose openGateCloseRedPose = new Pose(121, 75, Math.toRadians(270));
    public static Pose openGateFirstControlPointCloseRedPose = new Pose(109, 88, Math.toRadians(0));
    public static Pose openGateSecondControlPointCloseRedPose = new Pose(89, 68, Math.toRadians(0));

    public static Pose launchFirstCloseRedPose = new Pose(90, 90, Math.toRadians(330));

    public static Pose prepareForIntakeSecondCloseRedPose = new Pose(90, 54, Math.toRadians(0));

    public static Pose intakeSecondCloseRedPose = new Pose(128, 54, Math.toRadians(0));

    public static Pose launchSecondCloseRedPose = new Pose(90, 90, Math.toRadians(350));
    public static Pose launchSecondControlPointCloseRedPose = new Pose(86, 65, Math.toRadians(350));

    public static Pose prepareForIntakeThirdCloseRedPose = new Pose(90, 35, Math.toRadians(0));

    public static Pose intakeThirdCloseRedPose = new Pose(110, 35, Math.toRadians(0));

    public static Pose launchThirdCloseRedPose = new Pose(90, 90, Math.toRadians(330));

    public static Pose parkCloseRedPose = new Pose(104, 75, Math.toRadians(0));




    /// FURTHER

    public static Pose startingPoseFurtherRed = new Pose(87, 8.5, Math.toRadians(90));

    public static Pose prepareForIntakeFirstFurtherRedPose = new Pose(97, 35, Math.toRadians(0));
    public static Pose prepareForIntakeFirstControlPointFurtherRedPose = new Pose(85, 34, Math.toRadians(0));


    public static Pose intakeFirstFurtherRedPose = new Pose(124, 35, Math.toRadians(0));

    public static Pose launchFirstFurtherRedPose = new Pose(84, 17, Math.toRadians(0));

    public static Pose prepareForIntakeSecondFurtherRedPose = new Pose(131, 19, Math.toRadians(320));

    public static Pose intakeSecondFurtherRedPose = new Pose(135, 10, Math.toRadians(270));

    public static Pose launchSecondFurtherRedPose = new Pose(88, 15, Math.toRadians(340));




    public static Pose prepareForIntakeArtifactsFurtherRedPose = new Pose(102, 35, Math.toRadians(0));

    public static Pose intakeArtifactsFurtherRedPose = new Pose(129, 35, Math.toRadians(0));

    public static Pose launchSecondArtifactsFurtherRedPose = new Pose(87, 13, Math.toRadians(30));

    public static Pose parkFurtherRedPose = new Pose(90, 28, Math.toRadians(90));




    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ///  BLUE

    /// CLOSE

    public static Pose startingPoseCloseBlue = startingPoseCloseRed.mirror();

    public static Pose launchPreloadCloseBluePose = launchPreloadCloseRedPose.mirror();

    public static Pose prepareForIntakeFirstCloseBluePose = prepareForIntakeFirstCloseRedPose.mirror();

    public static Pose intakeFirstCloseBluePose = intakeFirstCloseRedPose.mirror();

    public static Pose openGateCloseBluePose = openGateCloseRedPose.mirror();
    public static Pose openGateFirstControlPointCloseBluePose = openGateFirstControlPointCloseRedPose.mirror();
    public static Pose openGateSecondControlPointCloseBluePose = openGateSecondControlPointCloseRedPose.mirror();


    public static Pose openGateControlPointCloseBluePose = openGateFirstControlPointCloseRedPose.mirror();

    public static Pose launchFirstCloseBluePose = launchFirstCloseRedPose.mirror();

    public static Pose prepareForIntakeSecondCloseBluePose = prepareForIntakeSecondCloseRedPose.mirror();

    public static Pose intakeSecondCloseBluePose = intakeSecondCloseRedPose.mirror();


    public static Pose launchSecondControlPointCloseBluePose = launchSecondControlPointCloseRedPose.mirror();

    public static Pose launchSecondCloseBluePose = launchSecondCloseRedPose.mirror();

    public static Pose prepareForIntakeThirdCloseBluePose = prepareForIntakeThirdCloseRedPose.mirror();

    public static Pose intakeThirdCloseBluePose = intakeThirdCloseRedPose.mirror();

    public static Pose launchThirdCloseBluePose = launchThirdCloseRedPose.mirror();

    public static Pose parkCloseBluePose = parkCloseRedPose.mirror();






    /// FURTHER

    public static Pose startingPoseFurtherBlue = startingPoseFurtherRed.mirror();

    public static Pose parkFurtherBluePose = parkFurtherRedPose.mirror();

}
