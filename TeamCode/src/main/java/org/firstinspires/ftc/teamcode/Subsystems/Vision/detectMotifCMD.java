package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.Subsystems.Turret.TurretSubsystem;
import org.firstinspires.ftc.teamcode.Utilities.Pattern;

@Config
public class detectMotifCMD extends CommandBase {

    private final VisionSubsystem visionSubsystem;

    private ElapsedTime timer;

    public detectMotifCMD(VisionSubsystem vSb) {
        visionSubsystem = vSb;

        addRequirements(visionSubsystem);
    }


    @Override
    public void initialize() {
        timer = new ElapsedTime();
       }


    @Override
    public void execute() {

        if (visionSubsystem.result != null && visionSubsystem.result.isValid() && !visionSubsystem.result.getFiducialResults().isEmpty()) {
            int id = visionSubsystem.result.getFiducialResults().get(0).getFiducialId();

            switch (id){
                case 21:
                    visionSubsystem.pattern = Pattern.GPP;
                    break;
                case 22:
                    visionSubsystem.pattern = Pattern.PGP;
                    break;
                case 23:
                    visionSubsystem.pattern = Pattern.PPG;
                    break;
            }


        }

    }

    @Override
    public boolean isFinished() {
        return timer.seconds() > 0.1;
    }
}
