package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import static org.firstinspires.ftc.teamcode.Subsystems.Vision.VisionSubsystem.limelightTaRatio;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsint0;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsint1;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsint2;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsint3;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsint4;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsint5;
import static org.firstinspires.ftc.teamcode.Utilities.StaticConstants.shoootVsout0;
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

    private ElapsedTime stopperTimmer;

    InterpLUT shooterPosLut = new InterpLUT();

    public InterpLUT vsFunc = new InterpLUT();

    Double TargetA = shoootVsint2;

    public shooterToBasketCMD(ShooterSubsystem shooterSb, VisionSubsystem vSb) {
        shooterSubsystem = shooterSb;
        visionSubsystem = vSb;

        vsFunc.add(shoootVsint0, shoootVsout0);
        vsFunc.add(shoootVsint1, shoootVsout1);
        vsFunc.add(shoootVsint2, shoootVsout2);
        vsFunc.add(shoootVsint3, shoootVsout3);
        vsFunc.add(shoootVsint4, shoootVsout4);
        vsFunc.add(shoootVsint5, shoootVsout5);
//generating final equation
        vsFunc.createLUT();

        addRequirements(shooterSubsystem);
    }


    @Override
    public void initialize() {
        stopperTimmer = new ElapsedTime();
        stopperTimmer.reset();
    }

    @Override
    public void execute() {
        TargetA = visionSubsystem.getAllianceTA();

        if (TargetA != null) {
            double targetA = Range.clip(TargetA, shoootVsint0 + 0.1, shoootVsint5 - 0.1);
            shooterSubsystem.shooterTarget = vsFunc.get(targetA);

        }

    }

}
