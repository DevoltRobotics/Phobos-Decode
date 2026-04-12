package org.firstinspires.ftc.teamcode.Subsystems.Sensors;

import static org.firstinspires.ftc.teamcode.Subsystems.Sensors.SensorsSubsystem.lightGreen;
import static org.firstinspires.ftc.teamcode.Subsystems.Sensors.SensorsSubsystem.lightPurple;
import static org.firstinspires.ftc.teamcode.Subsystems.Sensors.SensorsSubsystem.lightRed;

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
            if (sensorsSubsystem.targetArtifact == Artifact.Purple) {
                sensorsSubsystem.setLightPos(lightPurple);

            } else {
                sensorsSubsystem.setLightPos(lightGreen);

            }

        } else {
            if (sensorsSubsystem.laserState) {
                sensorsSubsystem.setLightPos(lightRed);

            }else if (tX != null && tA != null) {

                sensorsSubsystem.setLightPos(0.65);

            } else {
                sensorsSubsystem.setLightPos(0);

            }

        }
    }
}


