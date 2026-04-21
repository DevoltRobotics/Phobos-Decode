package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Utilities.Alliance;
import org.firstinspires.ftc.teamcode.Utilities.OpModeCommand;

@Disabled
public class FAR_TRAJ extends OpModeCommand {

    public FAR_TRAJ() {
        super(Alliance.RED, true);
    }

    private ElapsedTime timer = new ElapsedTime();
    Pose m(Pose p) {
        return Alliance.BLUE.equals(currentAlliance) ? p.mirror() : p;
    }

    private Path intakeFirst, park;
    private PathChain backIntakeFirst, launchFirst, prepareIntakeSeond, intakeSecond, launchSecond, pickCorner, pickCenter;


    Pose startingPose = m(new Pose(88.0, 8.2, Math.toRadians(0)));
    static Pose pick1Pose = new Pose(134.0, 8.2, Math.toRadians(0));
    static Pose backPick1Pose = new Pose(130.0, 8.2, Math.toRadians(0));

    static Pose shoot1Pose = new Pose(96, 8.2, Math.toRadians(25));
    static Pose preparePick2Pose = new Pose(100, 35.0, Math.toRadians(0));
    static Pose pick2Pose = new Pose(134.0, 35, Math.toRadians(0));
    static Pose shoot2Pose = new Pose(92.0, 10.0, Math.toRadians(0));

    static Pose pickCornerPose = new Pose(134.0, 9, Math.toRadians(0));

    static Pose pickCenterControlPoint = new Pose(93.0, 40.0, Math.toRadians(0));
    static Pose pickCenterPose = new Pose(134.0, 36, Math.toRadians(0));


    static Pose parkPose = new Pose(100.0, 25.0, Math.toRadians(0));

    Command autoCommand;

    public void createPaths() {

        intakeFirst = new Path(new BezierLine(startingPose, pick1Pose));
        intakeFirst.setTangentHeadingInterpolation();

        backIntakeFirst = follower.pathBuilder()
                .addPath(new BezierLine(
                        pick1Pose,
                        backPick1Pose))
                .addPath(new BezierLine(
                        backPick1Pose,
                        pick1Pose))
                .setConstantHeadingInterpolation(pick1Pose.getHeading())
                .setReversed()
                .build();

        launchFirst = follower.pathBuilder()
                .addPath(new BezierLine(
                        pick1Pose,
                        shoot1Pose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();


        prepareIntakeSeond = follower.pathBuilder()
                .addPath(new BezierLine(
                        preparePick2Pose,
                        pick2Pose))
                .setConstantHeadingInterpolation(
                        pick2Pose.getHeading())
                .build();

        intakeSecond = follower.pathBuilder()
                .addPath(new BezierLine(
                        preparePick2Pose,
                        pick2Pose))
                .setTangentHeadingInterpolation()
                .build();

        launchSecond = follower.pathBuilder()
                .addPath(new BezierLine(
                        pick2Pose,
                        shoot2Pose))
                .setConstantHeadingInterpolation(shoot2Pose.getHeading())
                .build();

        pickCorner = follower.pathBuilder()
                .addPath(new BezierLine(
                        shoot2Pose,
                        pickCornerPose))
                .setTangentHeadingInterpolation()
                .build();

        pickCenter = follower.pathBuilder()
                .addPath(new BezierLine(
                        shoot2Pose,
                        pickCenterPose))
                .setTangentHeadingInterpolation()
                .build();

        park = new Path(new BezierLine(shoot2Pose, parkPose));
        park.setConstantHeadingInterpolation(parkPose.getHeading());


    }

    @Override
    public void initialize() {
        if (currentAlliance.equals(Alliance.RED)) {
            follower.setStartingPose(startingPose);

        } else {
            follower.setStartingPose(startingPose.mirror());

        }

        createPaths();

        autoCommand =
                new ConditionalCommand(
                new SequentialCommandGroup(

                        pedroSb.followPathCmd(intakeFirst),
                        pedroSb.followPathCmd(backIntakeFirst),

                        pedroSb.followPathCmd(launchFirst),
                        pedroSb.followPathCmd(prepareIntakeSeond),

                        pedroSb.followPathCmd(intakeSecond),
                        pedroSb.followPathCmd(launchSecond),

                        new ConditionalCommand(
                                pedroSb.followPathCmd(pickCorner),
                                pedroSb.followPathCmd(pickCenter),
                                () -> visionSb.isArtifactsCorner()

                        ),

                        new ConditionalCommand(
                                pedroSb.followPathCmd(pickCorner),
                                pedroSb.followPathCmd(pickCenter),
                                () -> visionSb.isArtifactsCorner()

                        ),

                        new ConditionalCommand(
                                pedroSb.followPathCmd(pickCorner),
                                pedroSb.followPathCmd(pickCenter),
                                () -> visionSb.isArtifactsCorner()

                        ),

                        new ConditionalCommand(
                                pedroSb.followPathCmd(pickCorner),
                                pedroSb.followPathCmd(pickCenter),
                                () -> visionSb.isArtifactsCorner()

                        )),
                        pedroSb.followPathCmd(park),
                        ()-> timer.seconds() < 28

                );

    }

    @Override
    public void start() {
        timer.reset();
        autoCommand.schedule();

    }
}
