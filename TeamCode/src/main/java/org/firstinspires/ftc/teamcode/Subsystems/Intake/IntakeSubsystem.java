package org.firstinspires.ftc.teamcode.Subsystems.Intake;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSubsystem extends SubsystemBase {

    DcMotor intakeM;
    CRServo intakeS;

    DcMotor transferM;

    public double intakePower = 0;
    public double transferPower = 0;


    public Telemetry telemetry;

    //CONSTANTS

    public IntakeSubsystem(HardwareMap hMap) {
        intakeM = hMap.dcMotor.get("in");
        intakeM.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeS = hMap.get(CRServo.class,"star");
        intakeS.setDirection(DcMotorSimple.Direction.REVERSE);

        transferM = hMap.dcMotor.get("trans");

        intakeM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        transferM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    public void periodic() {
        intakeM.setPower(intakePower);

        intakeS.setPower(transferPower);
        transferM.setPower(transferPower);

    }
}
