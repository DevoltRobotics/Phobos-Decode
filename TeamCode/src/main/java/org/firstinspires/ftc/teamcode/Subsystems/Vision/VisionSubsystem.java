package org.firstinspires.ftc.teamcode.Subsystems.Vision;


import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utilities.Alliance;
import org.firstinspires.ftc.teamcode.Utilities.Pattern;

public class VisionSubsystem extends SubsystemBase {

    public Limelight3A ll;
    public static int limelightTaRatio = 100;

    public LLResult result;

    static double RIGHT_BOUND = 10;
    public Pattern pattern = Pattern.PGP;

    public final Alliance alliance;

    public boolean isAuto;
    public boolean closeAuto;


    public Telemetry telemetry;

    private boolean hasMovedServo = false;

    public enum llState {
        artifact,
        posEstimate

    }

    public llState llStatus;

    public VisionSubsystem(HardwareMap hMap, Telemetry telemetry, Alliance alliance, boolean isAuto, boolean closeAuto) {
        ll = hMap.get(Limelight3A.class, "limelight");

        this.alliance = alliance;

        this.closeAuto = closeAuto;

        this.isAuto = isAuto;

        this.telemetry = telemetry;

        ll.setPollRateHz(75);
        ll.start();

        ll.pipelineSwitch(2);


    }

    public void periodic() {
        result = ll.getLatestResult();

    }

    public Double getAllianceTA() {
        if (result != null && result.isValid() && !result.getFiducialResults().isEmpty()) {
            int id = result.getFiducialResults().get(0).getFiducialId();

            if (alliance == Alliance.ANY ||
                    (alliance == Alliance.RED && id == 24) ||
                    (alliance == Alliance.BLUE && id == 20)) {
                return result.getTa() * limelightTaRatio;
            }
        }

        return null;
    }


    public Double getAllianceTX() {

        if (result != null && result.isValid() && !result.getFiducialResults().isEmpty()) {
            int id = result.getFiducialResults().get(0).getFiducialId();

            if (alliance == Alliance.ANY ||
                    (alliance == Alliance.RED && id == 24) ||
                    (alliance == Alliance.BLUE && id == 20)) {
                return result.getTx();
            }
        }

        return null;
    }

    public Pattern getPatternDetected() {
        if (result != null && result.isValid() && !result.getFiducialResults().isEmpty()) {
            int id = result.getFiducialResults().get(0).getFiducialId();

            switch (id) {
                case 17:
                    return Pattern.GPP;
                case 18:
                    return Pattern.PGP;
                case 19:
                    return Pattern.PPG;

            }
        }

        return Pattern.PPG;
    }
}
