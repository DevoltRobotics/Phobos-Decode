package org.firstinspires.ftc.teamcode.Subsystems.Lifting;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class LiftingSubsystem extends SubsystemBase {

    DcMotorEx liftingM;


    public double power;


    public LiftingSubsystem(HardwareMap hMap) {

        liftingM = hMap.get(DcMotorEx.class, "lift");
    }

    @Override
    public void periodic() {

        liftingM.setPower(power);

    }
}
