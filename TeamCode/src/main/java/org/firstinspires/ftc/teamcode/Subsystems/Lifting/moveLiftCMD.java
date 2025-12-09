package org.firstinspires.ftc.teamcode.Subsystems.Lifting;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeSubsystem;

public class moveLiftCMD extends CommandBase {

    private final LiftingSubsystem liftingSb;

    Gamepad gamepad;
    public moveLiftCMD(LiftingSubsystem liftingSb, Gamepad gamepad) {
        this.gamepad = gamepad;

        this.liftingSb = liftingSb;

        addRequirements(liftingSb);
    }

    @Override
    public void execute() {
        liftingSb.power = gamepad.left_stick_y;
    }


}
