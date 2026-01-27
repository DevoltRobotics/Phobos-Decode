package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsint0;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsint1;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsint2;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsint3;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsint4;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsint5;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsint6;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsout0;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsout1;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsout2;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsout3;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsout4;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsout5;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsout6;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.Subsystems.Turret.TurretSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.VisionSubsystem;

import java.util.function.DoubleSupplier;

public class shooterToBasketCMD extends CommandBase {

    public static InterpLUT distanceLUT = new InterpLUT();

    public static InterpLUT vsFunc = new InterpLUT();

    private final ElapsedTime waitAimTimer;


    static {
        vsFunc.add(1, 1530);
        vsFunc.add(24.5, 1520);
        vsFunc.add(28.5, 1465);
        vsFunc.add(37.5, 1400);
        vsFunc.add(65, 1270);
        vsFunc.add(100, 1230);
        vsFunc.add(130, 1150);
        vsFunc.add(200, 1100);
        vsFunc.add(400, 1105);

        vsFunc.createLUT();

       /* distanceLUT.add(0, 0);
        distanceLUT.add(shoootVsint0, shoootVsout0);
        distanceLUT.add(shoootVsint1, shoootVsout1);
        distanceLUT.add(shoootVsint2, shoootVsout2);
        distanceLUT.add(shoootVsint3, shoootVsout3);
        distanceLUT.add(shoootVsint4, shoootVsout4);
        distanceLUT.add(shoootVsint5, shoootVsout5);
        distanceLUT.add(shoootVsint6, shoootVsout6);
        distanceLUT.add(1000000000, 1000000000);

        distanceLUT.createLUT();

        */
    }

    private final ShooterSubsystem shooterSb;

    private final VisionSubsystem visionSb;

    public shooterToBasketCMD(ShooterSubsystem shooterSb, VisionSubsystem visionSb) {
        this.shooterSb = shooterSb;

        this.visionSb = visionSb;

        waitAimTimer = new ElapsedTime();
        addRequirements(this.shooterSb);
    }

    @Override
    public void execute() {
        Double TargetA = visionSb.getAllianceTA();

        if (TargetA != null) {
            double targetA = Range.clip(TargetA, 24.5, 200);
            shooterSb.shooterTarget = vsFunc.get(targetA);

            waitAimTimer.reset();
        }/*else if (waitAimTimer.milliseconds() > 200){
            double tagSizeDistance = distanceLUT.get(turretSb.getDistanceToGoal());

            shooterSb.shooterTarget = vsFunc.get(tagSizeDistance);
        }*/else {
        }

    }

}
