package org.firstinspires.ftc.teamcode.Subsystems.Turret;

import static org.firstinspires.ftc.teamcode.Subsystems.Turret.TurretSubsystem.furtherCorrection;
import static org.firstinspires.ftc.teamcode.Subsystems.Turret.TurretSubsystem.manualIncrement;
import static org.firstinspires.ftc.teamcode.Utilities.Alliance.RED;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.Subsystems.Sensors.SensorsSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Sensors.lightTimeCMD;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.VisionSubsystem;

import java.util.function.BooleanSupplier;

@Config
public class turretToBasketCMD extends CommandBase {

    private final TurretSubsystem turretSb;
    private final VisionSubsystem visionSb;
    private final SensorsSubsystem sensorsSb;

    Follower follower;

    double newTurretTargetRelative = 0;

    BooleanSupplier isManual;
    BooleanSupplier isClose;

    Gamepad gamepad;

    enum distanceTurrret {
        CLOSE,
        FAR
    }

    public static int farZoneAngle = 30;
    public static int closeZoneAngle = 45;

    double goalHeading = 0;

    distanceTurrret distanceStatus = distanceTurrret.FAR;

    ElapsedTime waitAimTimer;
    ElapsedTime angleAimOffset;

    /*
    public turretToBasketCMD(TurretSubsystem turretSb, VisionSubsystem vSb, Follower fllw) {
        this.turretSb = turretSb;
        this.visionSb = vSb;

        follower = fllw;

        isManual = () -> false;

        waitAimTimer = new ElapsedTime();
        angleAimOffset = new ElapsedTime();

        addRequirements(this.turretSb);
    }

     */

    public turretToBasketCMD(TurretSubsystem turretSb, VisionSubsystem vSb, SensorsSubsystem sensorsSb, Follower fllw, BooleanSupplier isManual, BooleanSupplier isClose, Gamepad gamepad) {
        this.turretSb = turretSb;
        this.visionSb = vSb;
        this.sensorsSb = sensorsSb;

        follower = fllw;

        this.gamepad = gamepad;

        this.isManual = isManual;
        this.isClose = isClose;

        waitAimTimer = new ElapsedTime();
        angleAimOffset = new ElapsedTime();

        addRequirements(turretSb);
    }


    @Override
    public void execute() {
        Double tA = visionSb.getAllianceTA();

        Double tX = visionSb.getAllianceTX();

        double robotHeading = Math.toDegrees(follower.poseTracker.getPose().getHeading());

        turretSb.turretAbsolutepos = wrapAngle(robotHeading - turretSb.turretPRelative);

        double error = wrapAngle(goalHeading - turretSb.turretAbsolutepos);

        if (visionSb.alliance == RED) {
            goalHeading = isClose.getAsBoolean() ? (90 - closeZoneAngle) : (90 - farZoneAngle);
        } else {
            goalHeading = isClose.getAsBoolean() ? (90 + closeZoneAngle) : (90 + farZoneAngle);
        }

        if (gamepad.right_bumper || gamepad.left_bumper) {
            turretSb.realIsManual = true;

        } else {
            turretSb.realIsManual = isManual.getAsBoolean();

        }

        if (turretSb.realIsManual) {
            if (gamepad.right_bumper) {
                turretSb.turretTarget += manualIncrement;

            } else if (gamepad.left_bumper) {
                turretSb.turretTarget -= manualIncrement;

            }

        } else {
            if (tX != null && tA != null) {

                if (tA < 50) {
                    if (!visionSb.isAuto) {
                        switch (visionSb.alliance) {
                            case RED:
                                tX += furtherCorrection;
                                break;

                            case BLUE:
                                tX -= furtherCorrection;
                                break;
                        }
                    }
                }

                double llTarget = turretSb.llPidf.calculate(tX);
                turretSb.turretTarget -= llTarget;

                turretSb.telemetry.addData("llTarget", llTarget);

                waitAimTimer.reset();

            } /*else {
                double angleTarget = turretSb.anglePidController.calculate(error);

                turretSb.turretTarget += angleTarget;

                waitAimTimer.reset();
            }


           */
        }
    }

    private double wrapAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
}
