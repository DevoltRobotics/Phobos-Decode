package org.firstinspires.ftc.teamcode.Subsystems.Turret;

import static org.firstinspires.ftc.teamcode.Subsystems.Turret.TurretSubsystem.furtherCorrection;
import static org.firstinspires.ftc.teamcode.Subsystems.Turret.TurretSubsystem.turretPRelative;
import static org.firstinspires.ftc.teamcode.Utilities.Aliance.RED;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Sensors.SensorsSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.VisionSubsystem;

import java.util.function.BooleanSupplier;

@Config
public class turretDefaultTeleopCMD extends CommandBase {

    private final TurretSubsystem turretSb;

    private final VisionSubsystem visionSb;

    BooleanSupplier isManual;

    BooleanSupplier isClose;

    Gamepad gamepad;

    static int manualIncrement = 5;

    private final ElapsedTime waitAimTimer;

    public turretDefaultTeleopCMD(TurretSubsystem turretSb, VisionSubsystem visionSb, BooleanSupplier isManual, BooleanSupplier isClose, Gamepad gamepad) {
        this.turretSb = turretSb;

        this.visionSb = visionSb;

        this.gamepad = gamepad;

        this.isManual = isManual;

        this.isClose = isClose;

        waitAimTimer = new ElapsedTime();

        addRequirements(turretSb);
    }

    @Override
    public void execute() {


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

            if (tX > 7) {

                waitAimTimer.reset();

            }

            turretSb.telemetry.addData("llTarget", llTarget);


        } else if (waitAimTimer.milliseconds() > 150) {

            turretSb.turretTarget = turretSb.turretToGoalAngle;

        }


    }
}


