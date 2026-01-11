package org.firstinspires.ftc.teamcode.Subsystems.Turret;

import static org.firstinspires.ftc.teamcode.Subsystems.Turret.TurretSubsystem.furtherCorrection;
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

    BooleanSupplier isManual;
    BooleanSupplier isClose;

    Gamepad gamepad;

    public static int manualIncrement = 5;

    public turretDefaultTeleopCMD(TurretSubsystem turretSb, BooleanSupplier isManual, Gamepad gamepad) {
        this.turretSb = turretSb;

        this.gamepad = gamepad;

        this.isManual = isManual;

        addRequirements(turretSb);
    }

    @Override
    public void execute() {


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
            turretSb.turretTarget = turretSb.turretToGoalAngle;

        }
    }
}

