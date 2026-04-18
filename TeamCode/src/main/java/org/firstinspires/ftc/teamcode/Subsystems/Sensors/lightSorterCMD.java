package org.firstinspires.ftc.teamcode.Subsystems.Sensors;

import static org.firstinspires.ftc.teamcode.Subsystems.Sensors.SensorsSubsystem.lightBlue;
import static org.firstinspires.ftc.teamcode.Subsystems.Sensors.SensorsSubsystem.lightGreen;
import static org.firstinspires.ftc.teamcode.Subsystems.Sensors.SensorsSubsystem.lightPurple;
import static org.firstinspires.ftc.teamcode.Subsystems.Sensors.SensorsSubsystem.lightRed;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.VisionSubsystem;
import org.firstinspires.ftc.teamcode.Utilities.Artifact;
import org.firstinspires.ftc.teamcode.Utilities.Pattern;

import java.util.function.BooleanSupplier;

@Configurable
public class lightSorterCMD extends CommandBase {

    private final SensorsSubsystem sensorsSubsystem;

    private final ShooterSubsystem shooterSb;

    private final VisionSubsystem visionSb;

    private final BooleanSupplier preparingShoot;

    private final Gamepad gamepad;

    static double errorShooterLightTarget = 80;

    public lightSorterCMD(SensorsSubsystem sensorsSb, ShooterSubsystem shooterSb, VisionSubsystem visionSb, BooleanSupplier preparingShoot, Gamepad gamepad) {
        this.sensorsSubsystem = sensorsSb;

        this.shooterSb = shooterSb;
        this.visionSb = visionSb;

        this.preparingShoot = preparingShoot;

        this.gamepad = gamepad;
        addRequirements(sensorsSubsystem);
    }


    @Override
    public void execute() {

        Double tX = visionSb.getAllianceTX();
        Double tA = visionSb.getAllianceTA();

        if (sensorsSubsystem.sorterMode) {

            switch (sensorsSubsystem.teleOpPattern) {
                case PPG:
                    sensorsSubsystem.setLightPos(lightPurple, lightPurple);
                    break;
                case PGP:
                    sensorsSubsystem.setLightPos(lightPurple, lightGreen);
                    break;
                case GPP:
                    sensorsSubsystem.setLightPos(lightGreen, lightPurple);
                    break;

            }

        } else {
            if (sensorsSubsystem.laserState && gamepad.right_trigger > 0.5) {
                sensorsSubsystem.setLightPos(lightRed);

            }else if (preparingShoot.getAsBoolean() && Math.abs(shooterSb.shooterError) < errorShooterLightTarget) {
                sensorsSubsystem.setLightPos(lightBlue);

            } else {
                sensorsSubsystem.setLightPos(0);

            }

        }
    }
}


