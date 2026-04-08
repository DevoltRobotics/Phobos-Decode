package org.firstinspires.ftc.teamcode.Subsystems.Sensors;

import static org.firstinspires.ftc.teamcode.Subsystems.Sensors.SensorsSubsystem.lightGreen;
import static org.firstinspires.ftc.teamcode.Subsystems.Sensors.SensorsSubsystem.lightPurple;
import static org.firstinspires.ftc.teamcode.Subsystems.Sensors.SensorsSubsystem.lightRed;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.VisionSubsystem;
import org.firstinspires.ftc.teamcode.Utilities.Artifact;

public class lightSorterCMD extends CommandBase {

    private final SensorsSubsystem sensorsSubsystem;

    private final ShooterSubsystem shooterSb;

    private final VisionSubsystem visionSb;

    public lightSorterCMD(SensorsSubsystem sensorsSb, ShooterSubsystem shooterSb, VisionSubsystem visionSb) {
        sensorsSubsystem = sensorsSb;

        this.shooterSb = shooterSb;
        this.visionSb = visionSb;

        addRequirements(sensorsSubsystem);
    }


    @Override
    public void execute() {

        Double tX = visionSb.getAllianceTX();
        Double tA = visionSb.getAllianceTA();

        if (sensorsSubsystem.sorterMode) {
            if (sensorsSubsystem.laserState) {
                sensorsSubsystem.light.setPosition(lightRed);

            } else if (sensorsSubsystem.targetArtifact == Artifact.Purple) {
                sensorsSubsystem.light.setPosition(lightPurple);

            } else {
                sensorsSubsystem.light.setPosition(lightGreen);

            }

        } else {
            if (sensorsSubsystem.laserState) {
                sensorsSubsystem.light.setPosition(lightRed);

            }if (tX != null && tA != null) {

                sensorsSubsystem.light.setPosition(0.65);

            } else {
                sensorsSubsystem.light.setPosition(0);

            }

        }
    }
}


