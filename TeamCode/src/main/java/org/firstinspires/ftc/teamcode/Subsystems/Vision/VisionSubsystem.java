package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import static org.firstinspires.ftc.teamcode.PoseEstimate.ConversionUtil.from3DToPedro;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.PoseEstimate.wpilib.Units;
import org.firstinspires.ftc.teamcode.PoseEstimate.wpilib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.Utilities.Aliance;
import org.firstinspires.ftc.teamcode.Utilities.Pattern;

public class VisionSubsystem extends SubsystemBase {

    public Limelight3A ll;

    public static int limelightTaRatio = 100;

    public LLResult result;

    private static final int PPG_TAG_ID = 23;
    private static final int PGP_TAG_ID = 22;
    private static final int GPP_TAG_ID = 21;

    public Pattern pattern = Pattern.PGP;

    public final Aliance alliance;

    public boolean isAuto;

    public Telemetry telemetry;

    Follower follower;

    public VisionSubsystem(HardwareMap hMap, Telemetry telemetry, Aliance alliance, boolean isAuto, Follower follower) {
        ll = hMap.get(Limelight3A.class, "limelight");

        this.alliance = alliance;

        this.isAuto = isAuto;

        this.telemetry = telemetry;

        this.follower = follower;

        ll.setPollRateHz(100);
        ll.start();

        ll.pipelineSwitch(2);

    }

    public void periodic() {
        result = ll.getLatestResult();

        FtcDashboard.getInstance().getTelemetry().addData("llA", getAllianceTA());
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
