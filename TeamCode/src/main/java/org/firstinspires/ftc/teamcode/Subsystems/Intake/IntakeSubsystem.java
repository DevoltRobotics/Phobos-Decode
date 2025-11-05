package org.firstinspires.ftc.teamcode.Subsystems.Intake;

import android.graphics.Color;
import android.provider.CalendarContract;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Collections;
import java.util.Set;

public class IntakeSubsystem extends SubsystemBase {

    DcMotor intakeM;
    public double power = 0;

    public Telemetry telemetry;

    //CONSTANTS

    public IntakeSubsystem(HardwareMap hMap) {
        intakeM = hMap.dcMotor.get("in");

        intakeM.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void periodic() {
        intakeM.setPower(power);

    }
}
