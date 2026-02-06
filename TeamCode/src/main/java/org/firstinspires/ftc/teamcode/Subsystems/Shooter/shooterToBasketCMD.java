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
        vsFunc.add(250, 1060);
        vsFunc.add(400, 1060);

        vsFunc.createLUT();

       distanceLUT.add(-30, 399.5);
        distanceLUT.add(50, 399.5);
        distanceLUT.add(58.5, 270);
        distanceLUT.add(66.5, 191);
        distanceLUT.add(75, 140);
        distanceLUT.add(84, 110);
        distanceLUT.add(93.3, 82);
        distanceLUT.add(105, 63);

        distanceLUT.add(113, 51);
        distanceLUT.add(127, 40);
        distanceLUT.add(132, 35);

        distanceLUT.add(142, 30);

        distanceLUT.add(152, 26.5);

        distanceLUT.add(157, 24.6);

        distanceLUT.add(1000000000, 24.6);

        distanceLUT.createLUT();


    }

    private final ShooterSubsystem shooterSb;

    private final VisionSubsystem visionSb;

    private final TurretSubsystem turretSb;

    public shooterToBasketCMD(ShooterSubsystem shooterSb, VisionSubsystem visionSb, TurretSubsystem turretSb) {
        this.shooterSb = shooterSb;

        this.visionSb = visionSb;

        this.turretSb = turretSb;
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
        }/*else if (waitAimTimer.milliseconds() > 150){
            double tagSizeDistance = Range.clip(distanceLUT.get(turretSb.getDistanceToGoal()), 50.5, 154);

            shooterSb.shooterTarget = vsFunc.get(tagSizeDistance);
        }
        */



    }

}
