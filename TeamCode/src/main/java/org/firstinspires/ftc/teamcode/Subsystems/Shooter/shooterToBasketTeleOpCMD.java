package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import com.acmerobotics.dashboard.config.Config;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.Subsystems.Turret.TurretSubsystem;

@Config
public class shooterToBasketTeleOpCMD extends CommandBase {

    public static InterpLUT distanceLUT = new InterpLUT();

    public static int shooterVel = 1300;
    /*static {
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

     */

    static{
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

    private final TurretSubsystem turretSb;

    public shooterToBasketTeleOpCMD(ShooterSubsystem shooterSb, TurretSubsystem turretSb, Double provTarget) {
        this.shooterSb = shooterSb;

        this.turretSb = turretSb;

        addRequirements(this.shooterSb);
    }


    @Override
    public void execute() {

        //shooterSb.setShooterTarget(distanceLUT.get(turretSb.distanceToGoal));

        shooterSb.setShooterTarget(shooterVel);

    }

}
