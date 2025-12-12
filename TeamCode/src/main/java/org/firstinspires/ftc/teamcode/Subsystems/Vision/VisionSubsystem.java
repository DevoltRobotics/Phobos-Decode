package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utilities.Aliance;
import org.firstinspires.ftc.teamcode.Utilities.Pattern;

public class VisionSubsystem extends SubsystemBase {

    public Limelight3A limelight;

    public static int limelightTaRatio = 100;

    public LLResult result;

    private static final int PPG_TAG_ID = 23;
    private static final int PGP_TAG_ID = 22;
    private static final int GPP_TAG_ID = 21;

    public Pattern pattern = Pattern.GPP;

    public final Aliance alliance;

    public boolean isAuto;

    public Telemetry telemetry;

    public VisionSubsystem(HardwareMap hMap, Aliance alliance, Telemetry telemetry, boolean isAuto) {
        limelight = hMap.get(Limelight3A.class, "limelight");

        this.alliance = alliance;

        this.isAuto = isAuto;

        this.telemetry = telemetry;

        limelight.setPollRateHz(100);
        limelight.start();

        limelight.pipelineSwitch(0);




    }

    public void periodic(){
        result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            FtcDashboard.getInstance().getTelemetry().addData("LL AprilTag tA", result.getTa());
            FtcDashboard.getInstance().getTelemetry().addData("LL AprilTag tX", result.getTy());
        } else {
            FtcDashboard.getInstance().getTelemetry().addData("Limelight", "No Targets");
        }

        telemetry.addData("Pattern", pattern);
    }

    public Double getAllianceTA() {
        if (result != null && result.isValid() && !result.getFiducialResults().isEmpty()) {
            int id = result.getFiducialResults().get(0).getFiducialId();

            if (alliance == Aliance.ANY ||
                    (alliance == Aliance.RED && id == 24) ||
                    (alliance == Aliance.BLUE && id == 20)) {
                return result.getTa() * limelightTaRatio;
            }
        }

        return null;
    }


    public Double getAllianceTX() {

        if (result != null && result.isValid() && !result.getFiducialResults().isEmpty()) {
            int id = result.getFiducialResults().get(0).getFiducialId();

            if (alliance == Aliance.ANY ||
                    (alliance == Aliance.RED && id == 24) ||
                    (alliance == Aliance.BLUE && id == 20)) {
                return result.getTx();
            }
        }

        return null;
    }


    public Double getAllianceTXDegrees() {

        if (result != null && result.isValid() && !result.getFiducialResults().isEmpty()) {
            int id = result.getFiducialResults().get(0).getFiducialId();

            if (alliance == Aliance.ANY ||
                    (alliance == Aliance.RED && id == 24) ||
                    (alliance == Aliance.BLUE && id == 20)) {
                return result.getTxNC();
            }
        }

        return null;
    }
    public Pattern getPatternDetected() {
        if (result != null && result.isValid() && !result.getFiducialResults().isEmpty()) {
            int id = result.getFiducialResults().get(0).getFiducialId();

            switch (id){
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
