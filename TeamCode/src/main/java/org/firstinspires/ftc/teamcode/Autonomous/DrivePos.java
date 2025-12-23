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

    public static Pose launchPreloadCloseRedPose = new Pose(97, 97, Math.toRadians(40));

    public static Pose prepareForIntakeFirstCloseRedPose = new Pose(100, 89, Math.toRadians(0));

    public static Pose intakeFirstCloseRedPose = new Pose(130, 87, Math.toRadians(0));

    public static Pose openGateCloseRedPose = new Pose(124, 73, Math.toRadians(270));
    public static Pose openGateFirstControlPointCloseRedPose = new Pose(109, 88, Math.toRadians(0));
    public static Pose openGateSecondControlPointCloseRedPose = new Pose(89, 68, Math.toRadians(0));

    public static Pose launchFirstCloseRedPose = new Pose(94, 85, Math.toRadians(330));

    public static Pose prepareForIntakeSecondCloseRedPose = new Pose(90, 59, Math.toRadians(0));

    public static Pose intakeSecondCloseRedPose = new Pose(130, 59, Math.toRadians(0));

    public static Pose launchSecondCloseRedPose = new Pose(90, 101, Math.toRadians(330));
    public static Pose launchSecondControlPointCloseRedPose = new Pose(92, 65, Math.toRadians(350));

    public static Pose prepareForIntakeThirdCloseRedPose = new Pose(90, 35, Math.toRadians(0));

    public static Pose intakeThirdCloseRedPose = new Pose(110, 35, Math.toRadians(0));

    public static Pose launchThirdCloseRedPose = new Pose(90, 90, Math.toRadians(330));

    public static Pose parkCloseRedPose = new Pose(92, 115, Math.toRadians(0));



    /// FURTHER

    public static Pose startingPoseFurtherRed = new Pose(87, 8.5, Math.toRadians(90));

    public static Pose prepareForIntakeFirstFurtherRedPose = new Pose(97, 33, Math.toRadians(0));
    public static Pose prepareForIntakeFirstControlPointFurtherRedPose = new Pose(85, 32, Math.toRadians(0));

    public static Pose intakeFirstFurtherRedPose = new Pose(132, 32, Math.toRadians(0));
    public static Pose launchFirstFurtherRedPose = new Pose(88, 13, Math.toRadians(0));

    public static Pose prepareForIntakeSecondFurtherRedPose = new Pose(131, 36, Math.toRadians(290));

    public static Pose intakeSecondFurtherRedPose = new Pose(131, 10, Math.toRadians(270));

    public static Pose launchSecondFurtherRedPose = new Pose(88, 15, Math.toRadians(0));

    public static Pose parkFurtherRedPose = new Pose(98, 28, Math.toRadians(0));


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
    public static Pose startingPoseFurtherBlue =
            startingPoseFurtherRed.mirror();

    public static Pose prepareForIntakeFirstFurtherBluePose =
            prepareForIntakeFirstFurtherRedPose.mirror();

    public static Pose prepareForIntakeFirstControlPointFurtherBluePose =
            prepareForIntakeFirstControlPointFurtherRedPose.mirror();

    public static Pose intakeFirstFurtherBluePose =
            intakeFirstFurtherRedPose.mirror();

    public static Pose launchFirstFurtherBluePose =
            launchFirstFurtherRedPose.mirror();

    public static Pose prepareForIntakeSecondFurtherBluePose =
            prepareForIntakeSecondFurtherRedPose.mirror();

    public static Pose intakeSecondFurtherBluePose =
            intakeSecondFurtherRedPose.mirror();

    public static Pose launchSecondFurtherBluePose =
            launchSecondFurtherRedPose.mirror();

    public static Pose parkFurtherBluePose =
            parkFurtherRedPose.mirror();



}
