package org.firstinspires.ftc.teamcode.Subsystems.Lifting;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class LiftingSubsystem extends SubsystemBase {

    CRServo liftingS;

    public double power;

    public LiftingSubsystem(HardwareMap hMap) {

        liftingS = hMap.get(CRServo.class, "liftS");
    }

    @Override
    public void periodic() {

        liftingS.setPower(power);
    }

    public void setPower(double power){
        this.power = power;
    }
}
