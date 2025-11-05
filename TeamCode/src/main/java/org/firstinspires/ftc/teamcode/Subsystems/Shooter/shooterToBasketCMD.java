package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsint1;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsint2;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsint3;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsint4;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsint5;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsout1;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsout2;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsout3;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsout4;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsout5;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.Subsystems.Vision.VisionSubsystem;

public class shooterToBasketCMD extends CommandBase {

    private final ShooterSubsystem shooterSubsystem;
    private final VisionSubsystem visionSubsystem;

    private final Limelight3A limelight;

    private ElapsedTime stopperTimmer;

    Follower follower;
    
    InterpLUT shooterPosLut = new InterpLUT();

    public InterpLUT vsFunc = new InterpLUT();

    double goalX = 0, goalY = 144;

    public shooterToBasketCMD(ShooterSubsystem shooterSb, VisionSubsystem vSb, Follower fllw) {
        shooterSubsystem = shooterSb;
        visionSubsystem = vSb;

        follower = fllw;

        limelight = visionSubsystem.limelight;

        vsFunc.add(shoootVsint1, shoootVsout1);
        vsFunc.add(shoootVsint2, shoootVsout2);
        vsFunc.add(shoootVsint3, shoootVsout3);
        vsFunc.add(shoootVsint4, shoootVsout4);
        vsFunc.add(shoootVsint5, shoootVsout5);
//generating final equation
        vsFunc.createLUT();

        shooterPosLut.add(shoootVsint1, shoootVsout1);
        shooterPosLut.add(shoootVsint2, shoootVsout2);
        shooterPosLut.add(shoootVsint3, shoootVsout3);
        shooterPosLut.add(shoootVsint4, shoootVsout4);
//generating final equation
        shooterPosLut.createLUT();

        addRequirements(shooterSubsystem, visionSubsystem);
    }


    @Override
    public void initialize() {

        stopperTimmer.reset();
    }

    @Override
    public void execute() {

        if (shooterSubsystem.isBlueAliance) {
            goalX = 0;
        } else {
            goalX = 144;
        }

        double posX = follower.getPose().getX();
        double posY = follower.getPose().getY();

        double distance = Math.hypot(goalX - posX, goalY - posY);

        LLResult result = limelight.getLatestResult();

        if (result != null) {
            if (result.isValid()) {
               shooterSubsystem.targetdistance = Range.clip(vsFunc.get(result.getTa()), shoootVsint1 + 0.1, shoootVsint5 - 0.1);
            }

            else {
                shooterSubsystem.targetdistance =  vsFunc.get(shoootVsint1 + 0.1);

            }
        }

        shooterSubsystem.shooterTarget = shooterSubsystem.targetdistance;
    }

    @Override
    public boolean isFinished() {
        return visionSubsystem.limelight.getLatestResult().isValid() || stopperTimmer.seconds() > 0.5;
    }
}
