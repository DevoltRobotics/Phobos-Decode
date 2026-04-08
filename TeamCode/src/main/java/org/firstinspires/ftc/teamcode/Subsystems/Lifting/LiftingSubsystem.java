package org.firstinspires.ftc.teamcode.Subsystems.Lifting;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import java.util.function.BooleanSupplier;

@Configurable
public class LiftingSubsystem extends SubsystemBase {

    Servo liftingS;

    DcMotorEx fr, fl;

    public double PTOPos;

    public double liftPower;

    public static double ptoOff = 0.5;
    public static double ptoOn = 0.75;

    BooleanSupplier isLiftingSupplier;

    public LiftingSubsystem(HardwareMap hMap, BooleanSupplier isLiftingSupplier) {

        liftingS = hMap.get(Servo.class, "liftS");

        fr = hMap.get(DcMotorEx.class, "fr");
        fl = hMap.get(DcMotorEx.class, "fl");

        this.isLiftingSupplier = isLiftingSupplier;

    }

    @Override
    public void periodic() {
/*
        if (isLiftingSupplier.getAsBoolean()){
            liftingS.setPosition(ptoOn);
            fr.setPower(liftPower);
            fl.setPower(-liftPower);
        }else {
            liftingS.setPosition(ptoOff);
        }

 */
    }

    public void setPtoPosition(double position){
        liftingS.setPosition(position);
    }

    public void setLiftPower(double power){
        liftPower = power;

    }
}
