package org.firstinspires.ftc.teamcode.Subsystems.Sensors;

import static org.firstinspires.ftc.teamcode.Subsystems.Turret.TurretSubsystem.furtherCorrection;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Turret.TurretSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.VisionSubsystem;
import org.firstinspires.ftc.teamcode.Utilities.Artifact;

public class lightSorterCMD extends CommandBase {

    private final SensorsSubsystem sensorsSubsystem;
    private final TurretSubsystem turretSb;
    private final ShooterSubsystem shooterSb;

    public lightSorterCMD(SensorsSubsystem sensorsSb, TurretSubsystem turretSb, ShooterSubsystem shooterSb) {
        sensorsSubsystem = sensorsSb;

        this.turretSb = turretSb;
        this.shooterSb = shooterSb;

        addRequirements(sensorsSubsystem);
    }


    @Override
    public void execute() {

        if (sensorsSubsystem.sorterMode) {
            if (sensorsSubsystem.fourDetected) {
                sensorsSubsystem.light.setPosition(0.28);


            } else if (sensorsSubsystem.targetArtifact == Artifact.Purple) {
                sensorsSubsystem.light.setPosition(0.7);

            } else {
                sensorsSubsystem.light.setPosition(0.5);

            }

        } else {

            if (sensorsSubsystem.laserState) {
                sensorsSubsystem.light.setPosition(0.33);

            } else {
                sensorsSubsystem.light.setPosition(0);

            }


        }


    }


}


