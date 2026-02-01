package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.Subsystems.Turret.TurretSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.VisionSubsystem;

public class shooterToBasketTeleOpCMD extends CommandBase {

    public static InterpLUT distanceLUT = new InterpLUT();

    public static InterpLUT vsFunc = new InterpLUT();

    private final ElapsedTime waitAimTimer;

    private Double provTarget;


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

    public shooterToBasketTeleOpCMD(ShooterSubsystem shooterSb, VisionSubsystem visionSb, TurretSubsystem turretSb, Double provTarget) {
        this.shooterSb = shooterSb;

        this.visionSb = visionSb;

        this.turretSb = turretSb;

        this.provTarget = provTarget;

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
        }else if (waitAimTimer.milliseconds() > 200){
            shooterSb.shooterTarget = provTarget;

        }

        /*else if (waitAimTimer.milliseconds() > 150){
            double tagSizeDistance = Range.clip(distanceLUT.get(turretSb.getDistanceToGoal()), 50.5, 154);

            shooterSb.shooterTarget = vsFunc.get(tagSizeDistance);
        }
        */



    }

}
