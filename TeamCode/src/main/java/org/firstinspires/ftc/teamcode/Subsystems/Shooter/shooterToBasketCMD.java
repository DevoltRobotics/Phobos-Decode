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
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.Subsystems.Vision.VisionSubsystem;
import org.firstinspires.ftc.teamcode.Utilities.Aliance;

public class shooterToBasketCMD extends CommandBase {

    public static InterpLUT vsFunc = new InterpLUT();

    public static InterpLUT distanceFunc = new InterpLUT();

    Aliance aliance;
    private double goalX = 0;
    double goalY = 140;

    /*static {
        vsFunc.add(shoootVsint0, shoootVsout0);
        vsFunc.add(shoootVsint1, shoootVsout1);
        vsFunc.add(shoootVsint2, shoootVsout2);
        vsFunc.add(shoootVsint3, shoootVsout3);
        vsFunc.add(shoootVsint4, shoootVsout4);
        vsFunc.add(shoootVsint5, shoootVsout5);
        vsFunc.add(shoootVsint6, shoootVsout6);
//generating final equation
        vsFunc.createLUT();
    }*/

    static {
        distanceFunc.add(shoootVsint0, shoootVsout0);
        distanceFunc.add(shoootVsint1, shoootVsout1);
        distanceFunc.add(shoootVsint2, shoootVsout2);
        distanceFunc.add(shoootVsint3, shoootVsout3);
        distanceFunc.add(shoootVsint4, shoootVsout4);
        distanceFunc.add(shoootVsint5, shoootVsout5);
        distanceFunc.add(shoootVsint6, shoootVsout6);

        distanceFunc.createLUT();
    }
    private final ShooterSubsystem shooterSubsystem;

    private double targetEmergency;

    public shooterToBasketCMD(ShooterSubsystem shooterSb, Aliance aliance) {
        shooterSubsystem = shooterSb;

        this.aliance = aliance;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        goalX = aliance.equals(Aliance.RED) ? 140 : 4;
    }

    @Override
    public void execute() {

        double robotX = follower.getPose().getX();
        double robotY = follower.getPose().getY();

        double dx = goalX - robotX;
        double dy = goalY - robotY;

        double distanceToGoal = Math.hypot(dx, dy); // distance in field units

        shooterSubsystem.shooterTarget = distanceFunc.get(distanceToGoal);


    }

}
