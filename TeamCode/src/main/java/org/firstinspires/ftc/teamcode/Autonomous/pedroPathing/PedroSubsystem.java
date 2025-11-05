package org.firstinspires.ftc.teamcode.Autonomous.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PedroSubsystem extends SubsystemBase {

    public Follower follower;

    public Pose EndPose = new Pose();

    Telemetry telemetry;

    public PedroSubsystem(Follower follower, Telemetry telemetry) {
        this.follower = follower;

        this.telemetry = telemetry;
        
    }

    @Override
    public void periodic() {
        follower.update();

        EndPose = follower.getPose();
    }

    public Command followPathCmd(Path path) {
        return new FollowPathCmd(path);
    }

    public Command followPathCmd(PathChain path) {
        return new FollowPathChainCmd(path);
    }

    public Command fieldCentricCmd(Gamepad gamepad) {
        return new FieldCentricCmd(gamepad);
    }

    class FieldCentricCmd extends CommandBase {
        Gamepad gamepad;

        FieldCentricCmd(Gamepad gamepad) {
            this.gamepad = gamepad;
            addRequirements(PedroSubsystem.this);
        }

        @Override
        public void initialize() {
            follower.startTeleopDrive();
        }

        @Override
        public void execute() {
            follower.setTeleOpDrive(-gamepad.left_stick_y, -gamepad.left_stick_x, -gamepad.right_stick_x, false);
        }

        @Override
        public void end(boolean interrupted) {
            follower.breakFollowing();
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
            follower.followPath(path, true);
        }

        @Override
        public boolean isFinished() {
            return !follower.isBusy();
        }


        public Command fieldCentricCmd(Gamepad gamepad) {
            return new FieldCentricCmd(gamepad);
        }


        class FieldCentricCmd extends CommandBase {
            Gamepad gamepad;

            FieldCentricCmd(Gamepad gamepad) {
                this.gamepad = gamepad;
                addRequirements(PedroSubsystem.this);
            }

            @Override
            public void initialize() {
                follower.startTeleopDrive();
            }

            @Override
            public void execute() {
                follower.setTeleOpDrive(
                        -gamepad.left_stick_y,
                        -gamepad.left_stick_x,
                        -gamepad.right_stick_x,
                        false // Robot Centric
                );
            }

            @Override
            public void end(boolean interrupted) {
                follower.breakFollowing();
            }

            @Override
            public boolean isFinished() {
                return false;
            }
        }
    }
}