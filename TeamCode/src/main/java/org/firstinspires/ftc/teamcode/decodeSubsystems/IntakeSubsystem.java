package org.firstinspires.ftc.teamcode.decodeSubsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robocol.Command;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class IntakeSubsystem extends SubsystemBase {
    private final DcMotorEx motor;

    private final Servo intakeServo1, intakeServo2;
    private final double openedPos = 0.7, closedPos = 0.3;

    public IntakeSubsystem(HardwareMap hm) {
        motor = hm.get(DcMotorEx.class, "intakeMotor");
        intakeServo1 = hm.get(Servo.class, "rightIntake");
        intakeServo2 = hm.get(Servo.class, "leftIntake");
    }

    public void setPower(double power) {
        motor.setPower(power);
    }

    public void setPosition(double position) {
        intakeServo1.setPosition(position);
        intakeServo2.setPosition(1 - position);
    }
    public double getCurrent(){
        return motor.getCurrent(CurrentUnit.AMPS);
    }


}
