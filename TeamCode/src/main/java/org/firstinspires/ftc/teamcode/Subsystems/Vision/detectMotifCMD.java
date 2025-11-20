package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Utilities.Pattern;

@Config
public class detectMotifCMD extends CommandBase {

    private final VisionSubsystem visionSubsystem;

    private ElapsedTime timer;


    public detectMotifCMD(VisionSubsystem vSb) {
        visionSubsystem = vSb;
        visionSubsystem.limelight.pipelineSwitch(1);

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
    public void end(boolean interrupted) {
        visionSubsystem.limelight.pipelineSwitch(0);
    }

    @Override
    public boolean isFinished() {
        return timer.seconds() > 1;
    }
}
