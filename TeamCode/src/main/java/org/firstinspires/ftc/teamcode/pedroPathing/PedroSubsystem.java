package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utilities.Alliance;

@Configurable
public class PedroSubsystem extends SubsystemBase {

    public Follower follower;

    public static double slowModeMultiplier = 0.4;
    public static double xBlueFarCorner = 7.5;
    public static double yBlueFarCorner = 8.075;
    public static double xRedFarCorner = 144 - xBlueFarCorner;
    public static double yRedFarCorner = yBlueFarCorner;

    public static double xBlueCloseCorner = 14.5;
    public static double yBlueCloseCorner = 106.925;
    public static double xRedCloseCorner = 144 - xBlueCloseCorner;
    public static double yRedCloseCorner = yBlueCloseCorner;


    Telemetry telemetry;

    public static Pose EndPose = new Pose();

    Alliance alliance;

    public PedroSubsystem(Follower follower, Telemetry telemetry, Alliance alliance) {
        this.follower = follower;

        this.telemetry = telemetry;
        this.alliance = alliance;

    }

    @Override
    public void periodic() {
        /*telemetryM.debug("x:" + follower.getPose().getX());
        telemetryM.debug("y:" + follower.getPose().getY());
        telemetryM.debug("heading:" + follower.getPose().getHeading());
        telemetryM.debug("total heading:" + follower.getTotalHeading());

         */

    }

    public Command followPathCmd(Path path) {
        return new FollowPathCmd(path);
    }


    public Command followPathCmd(PathChain path) {
        return new FollowPathChainCmd(path);
    }
    public Command followPathCmd(PathChain path, double maxPower) {
        return new FollowPathChainCmd(path, maxPower);
    }


    public Command fieldCentricCmd(Gamepad gamepad, double angleOffset) {
        return new FieldCentricCmd(gamepad, angleOffset);
    }

    public Command turnToCmd(double heading) {
        return new TurnToCommand(heading);
    }

    class FieldCentricCmd extends CommandBase {
        Gamepad gamepad;

        double angleOffset;

        FieldCentricCmd(Gamepad gamepad, double angleOffset) {
            this.gamepad = gamepad;

            this.angleOffset = angleOffset;
            addRequirements(PedroSubsystem.this);
        }

        @Override
        public void initialize() {

            follower.startTeleopDrive();
        }

        @Override
        public void execute() {
            double multiplier = 1;

            if (gamepad.right_trigger >= 0.7) {
                multiplier = slowModeMultiplier;
            }

            follower.setTeleOpDrive(
                    -gamepad.left_stick_y * multiplier,
                    -gamepad.left_stick_x * multiplier,
                    -gamepad.right_stick_x * multiplier,
                    false,
                    angleOffset// Robot Centric
            );

            /*if (gamepad.yWasPressed()) {
                follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), angleOffset));
            }

             */
        }

        @Override
        public void end(boolean interrupted) {
            //follower.breakFollowing();
        }

        @Override
        public boolean isFinished() {
            return false;
        }
    }

    class FollowPathCmd extends CommandBase {
        Path path;

        FollowPathCmd(Path path) {
            this.path = path;

            addRequirements(PedroSubsystem.this);
        }

        @Override
        public void initialize() {
            follower.followPath(path);

        }

        @Override
        public boolean isFinished() {

            return !follower.isBusy();

        }
    }

    class FollowPathChainCmd extends CommandBase {
        PathChain path;

        double maxPower;

        FollowPathChainCmd(PathChain path) {
            this.path = path;
            this.maxPower = 1;

            addRequirements(PedroSubsystem.this);
        }

        FollowPathChainCmd(PathChain path, double maxPower) {
            this.path = path;
            this.maxPower = maxPower;

            addRequirements(PedroSubsystem.this);
        }

        @Override
        public void initialize() {
            follower.followPath(path, maxPower, true);

        }

        @Override
        public boolean isFinished() {

            return !follower.isBusy();

        }
    }

    class TurnToCommand extends CommandBase {

        double heading;

        TurnToCommand(double heading) {
            this.heading = heading;
            addRequirements(PedroSubsystem.this);
        }

        @Override
        public void initialize() {
            follower.turnTo(Math.toRadians(heading));
        }

        @Override
        public boolean isFinished() {
            return !follower.isBusy();
        }
    }

}
