package org.firstinspires.ftc.teamcode.Subsystems.Turret;

import static org.firstinspires.ftc.teamcode.Subsystems.Turret.TurretSubsystem.furtherCorrection;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Vision.VisionSubsystem;

import java.util.function.BooleanSupplier;

@Config
public class turretDefaultTeleopCMD extends CommandBase {

    private final TurretSubsystem turretSb;

    private final VisionSubsystem visionSb;

    BooleanSupplier isManual;

    BooleanSupplier isClose;

    BooleanSupplier isShooting;

    BooleanSupplier isPoseEstimate;

    Gamepad gamepad;

    static int manualIncrement = 5;

    private final ElapsedTime waitAimTimer;

    public turretDefaultTeleopCMD(TurretSubsystem turretSb, VisionSubsystem visionSb, BooleanSupplier isManual, BooleanSupplier isClose, Gamepad gamepad, BooleanSupplier isShooting, BooleanSupplier isPoseEstimate) {
        this.turretSb = turretSb;

        this.visionSb = visionSb;

        this.gamepad = gamepad;

        this.isManual = isManual;

        this.isClose = isClose;

        this.isShooting = isShooting;

        this.isPoseEstimate = isPoseEstimate;


        waitAimTimer = new ElapsedTime();

        addRequirements(turretSb);
    }

    @Override
    public void execute() {

        /*double gx = Math.cos(turretSb.goalAngleRad);
        double gy = Math.sin(turretSb.goalAngleRad);

        double px = -gy;
        double py =  gx;

        double lateralVelocity = turretSb.velX * px + turretSb.velY * py;

        double motionFF = lateralVelocity * TurretSubsystem.kBotToTurretVel;


         */
        if (gamepad.right_bumper || gamepad.left_bumper) {
            turretSb.realIsManual = true;

        } else {
            turretSb.realIsManual = isManual.getAsBoolean();

        }

        Double tA = visionSb.getAllianceTA();

        Double tX = visionSb.getAllianceTX();

        if (turretSb.realIsManual) {
            if (gamepad.right_bumper) {
                waitAimTimer.reset();
                turretSb.turretTarget += manualIncrement;

            } else if (gamepad.left_bumper) {
                waitAimTimer.reset();
                turretSb.turretTarget -= manualIncrement;

            }

        } else if (tX != null && tA != null) {

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

            if (isShooting.getAsBoolean()) {
                waitAimTimer.reset();
            }

            turretSb.telemetry.addData("llTarget", llTarget);

        }

        else if (isPoseEstimate.getAsBoolean() && waitAimTimer.milliseconds() > 250) {

            turretSb.turretTarget = turretSb.turretToGoalAngle;

        }
    }
}


