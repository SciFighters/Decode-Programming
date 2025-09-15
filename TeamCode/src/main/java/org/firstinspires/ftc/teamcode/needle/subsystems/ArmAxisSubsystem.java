package org.firstinspires.ftc.teamcode.needle.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class ArmAxisSubsystem extends SubsystemBase {
    public final double ticksPerDegree = -487.0 / 9.0;
    public final double maxRotation = 135;
    public final double basketAngle = 110;
    DcMotorEx motor;


    public ArmAxisSubsystem(HardwareMap hm) {
        motor = hm.get(DcMotorEx.class, "axisMotor");

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
//        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }


    public double getAngle() {
        return motor.getCurrentPosition() / ticksPerDegree;
//        return startAngle + motor.getCurrentPosition() / ticksPerDegree;
    }

    public void setAxisPower(double power) {
        if (getAngle() <= 0) {
            motor.setPower(Range.clip(power, 0, 1));
        } else if (getAngle() >= maxRotation) {
            motor.setPower(Range.clip(power, -1, 0));
        } else {
            motor.setPower(power);
        }
    }

    public void setRawPower(double power) {
        motor.setPower(power);
    }

    public double getCurrent() {
        return motor.getCurrent(CurrentUnit.AMPS);
    }
    public double getPower(){
        return motor.getPower();
    }
    public void breakMode(){
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void gotoMode(){
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

}
