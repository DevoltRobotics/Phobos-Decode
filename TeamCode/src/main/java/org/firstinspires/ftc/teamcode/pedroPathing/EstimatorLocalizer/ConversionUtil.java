package org.firstinspires.ftc.teamcode.pedroPathing.EstimatorLocalizer;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
public class ConversionUtil {


    public static Pose from3DToPedro(Pose3D pose3d, double headingRad) {
        double xFtc = pose3d.getPosition().x * 39.37;
        double yFtc = pose3d.getPosition().y * 39.37;
        // 1) shift origin: center → corner
        double xShifted = 72 - xFtc;
        double yShifted = yFtc + 72.0;

        // 2) axis swap
        double xPedro = yShifted;
        double yPedro = xShifted;

        // 3) heading conversion
        double headingPedro = -headingRad;

        return new Pose(
                xPedro,
                yPedro,
                headingPedro
        );
    }

}
