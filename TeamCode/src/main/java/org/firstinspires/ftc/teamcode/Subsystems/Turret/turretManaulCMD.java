package org.firstinspires.ftc.teamcode.Subsystems.Turret;

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
    public void initialize() {

    }

    @Override
    public void execute() {

        boolean isManual = gamepad.right_bumper || gamepad.left_bumper;

        if (isManual){
            turretSubsystem.isTurretManual = true;

            if (gamepad.right_bumper) {
                turretSubsystem.turretPower = 0.85;
            } else if (gamepad.left_bumper) {
                turretSubsystem.turretPower = -0.85;
            }

        }else {
            turretSubsystem.isTurretManual = false;
            turretSubsystem.turretPower = 0;
        }

    }

    @Override
    public boolean isFinished() {
        turretSubsystem.isTurretManual = false;

        return false;
    }

}
