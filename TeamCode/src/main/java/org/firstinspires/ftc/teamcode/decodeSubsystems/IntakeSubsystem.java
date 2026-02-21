package org.firstinspires.ftc.teamcode.decodeSubsystems;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robocol.Command;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class IntakeSubsystem extends SubsystemBase {
    private final DcMotorEx motor;
    private final Servo intakeServo1, intakeServo2;

    public static boolean reversed = false;

    public IntakeSubsystem(HardwareMap hm) {
        motor = hm.get(DcMotorEx.class, "intakeMotor");
        intakeServo1 = hm.get(Servo.class, "rightIntake");
        intakeServo2 = hm.get(Servo.class, "leftIntake");

        reversed = false;
    }

    public void setPower(double power) {
        motor.setPower(power);
    }

    public void setPosition(double position) {
        intakeServo1.setPosition(position);
        intakeServo2.setPosition(1 - position);
    }

    public double getCurrent() {
        return motor.getCurrent(CurrentUnit.AMPS);
    }


}
