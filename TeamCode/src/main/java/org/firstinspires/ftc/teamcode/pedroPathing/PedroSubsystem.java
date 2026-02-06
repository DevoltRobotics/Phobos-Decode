package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.telemetryM;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Configurable
@Config
public class PedroSubsystem extends SubsystemBase {

    public Follower follower;

    public static double slowModeMultiplier = 0.4;

    Telemetry telemetry;

    TelemetryManager telemetryM;

    public static Pose EndPose = new Pose();


    public PedroSubsystem(Follower follower, Telemetry telemetry, TelemetryManager telemetryM) {
        this.follower = follower;

        this.telemetry = telemetry;
        this.telemetryM = telemetryM;
    }

    @Override
    public void periodic() {
        /*telemetryM.debug("x:" + follower.getPose().getX());
        telemetryM.debug("y:" + follower.getPose().getY());
        telemetryM.debug("heading:" + follower.getPose().getHeading());
        telemetryM.debug("total heading:" + follower.getTotalHeading());

        telemetryM.update(telemetry);


         */
        follower.update();
    }

    public Command followPathCmd(Path path) {
        return new FollowPathCmd(path);
    }


    public Command followPathCmd(PathChain path) {
        return new FollowPathChainCmd(path);
    }


    public Command fieldCentricCmd(Gamepad gamepad, double angleOffset) {
        return new FieldCentricCmd(gamepad, angleOffset);
    }

    public Command turnToCmd(Pose pose) {
        return new TurnToCommand(pose);

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

            if (gamepad.yWasPressed()) {
                follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), angleOffset));
            }
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

        FollowPathChainCmd(PathChain path) {
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

    class TurnToCommand extends CommandBase {

        Pose pose;

        TurnToCommand(Pose pose) {
            this.pose = pose;
            addRequirements(PedroSubsystem.this);
        }

        @Override
        public void initialize() {
            follower.turnTo(pose.getHeading());
        }

        @Override
        public boolean isFinished() {
            return !follower.isBusy();
        }
    }

}
