package org.firstinspires.ftc.teamcode.Subsystems.Lifting;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Intake.IntakeSubsystem;

public class moveLiftCMD extends CommandBase {

    private final LiftingSubsystem liftingSb;

    double power;
    public moveLiftCMD(LiftingSubsystem liftingSb, double power) {

        this.liftingSb = liftingSb;

        this.power = power;

        addRequirements(liftingSb);
    }

    @Override
    public void execute() {
        liftingSb.setPower(power);
    }

    @Override
    public void end(boolean interrupted) {
        liftingSb.setPower(power);
    }
}
