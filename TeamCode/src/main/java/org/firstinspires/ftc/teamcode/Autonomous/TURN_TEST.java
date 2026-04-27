package org.firstinspires.ftc.teamcode.Autonomous;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Utilities.Alliance;
import org.firstinspires.ftc.teamcode.Utilities.OpModeCommand;

@Autonomous
@Configurable
public class TURN_TEST extends OpModeCommand {

    public TURN_TEST() {
        super(Alliance.RED, true, true);
    }

    public static double p = 0.02;
    public static double d = 0.08;

    public static double f = 0.025;

    public static int heading = 30;


    private Pose currentStartingPose;
    Command autoCommand;

    @Override
    public void initialize() {
        follower.setStartingPose(new Pose());

        autoCommand =
                new SequentialCommandGroup(

                        pedroSb.turnToCmd(heading),
                        pedroSb.turnToCmd(-heading)
                );

    }

    @Override
    public void start() {
        autoCommand.schedule();
    }
}

