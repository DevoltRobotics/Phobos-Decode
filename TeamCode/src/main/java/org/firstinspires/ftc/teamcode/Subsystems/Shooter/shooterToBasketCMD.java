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

import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.Subsystems.Turret.TurretSubsystem;

public class shooterToBasketCMD extends CommandBase {

    public static InterpLUT distanceLUT = new InterpLUT();

    static {
        distanceLUT.add(0, 0);
        distanceLUT.add(shoootVsint0, shoootVsout0);
        distanceLUT.add(shoootVsint1, shoootVsout1);
        distanceLUT.add(shoootVsint2, shoootVsout2);
        distanceLUT.add(shoootVsint3, shoootVsout3);
        distanceLUT.add(shoootVsint4, shoootVsout4);
        distanceLUT.add(shoootVsint5, shoootVsout5);
        distanceLUT.add(shoootVsint6, shoootVsout6);
        distanceLUT.add(1000000000, 1000000000);


        distanceLUT.createLUT();
    }

    double distance;

    private final ShooterSubsystem shooterSb;

    private final TurretSubsystem turretSb;

    public shooterToBasketCMD(ShooterSubsystem shooterSb, TurretSubsystem turretSb) {
        this.shooterSb = shooterSb;
        this.turretSb = turretSb;

        addRequirements(this.shooterSb);
    }

    @Override
    public void execute() {

        distance = turretSb.getDistanceToGoal();

        double velocity = distanceLUT.get(distance);

        shooterSb.shooterTarget = (Range.clip(velocity, 1150,1600));

        shooterSb.shooterTarget = distanceLUT.get(distance);

    }

}
