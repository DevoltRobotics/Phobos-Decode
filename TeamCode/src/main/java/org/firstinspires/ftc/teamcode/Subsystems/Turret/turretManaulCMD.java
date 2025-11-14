package org.firstinspires.ftc.teamcode.Subsystems.Turret;

import static org.firstinspires.ftc.teamcode.Subsystems.Turret.TurretSubsystem.manualIncrement;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.CommandBase;

public class turretManaulCMD extends CommandBase {

    private final TurretSubsystem turretSubsystem;

    private Gamepad gamepad;

    public turretManaulCMD(TurretSubsystem turretSb, Gamepad gamepad) {
        turretSubsystem = turretSb;
        this.gamepad = gamepad;

        addRequirements(turretSubsystem);
    }

    @Override
    public void execute() {
        if (gamepad.right_bumper) {
            turretSubsystem.turretTarget += manualIncrement;
        } else if (gamepad.left_bumper) {
            turretSubsystem.turretTarget -= manualIncrement;
        }

    }
}
