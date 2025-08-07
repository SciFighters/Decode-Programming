package org.firstinspires.ftc.teamcode.needle.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class ArmAxisSubsystem extends SubsystemBase {
    public final double ticksPerDegree = 43.2;
    private double startAngle = 0;
    public final double maxRotation = 135 * ticksPerDegree;
    public final double basketAngle = 110;
    DcMotorEx motor;

    public ArmAxisSubsystem(HardwareMap hm) {
        motor = hm.get(DcMotorEx.class, "axisMotor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public ArmAxisSubsystem(HardwareMap hm, double startAngle) {
        motor = hm.get(DcMotorEx.class, "axisMotor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.startAngle = startAngle;
    }

    public double getAngle() {
        return motor.getCurrentPosition();
//        return startAngle + motor.getCurrentPosition() / ticksPerDegree;
    }

    public void setAxisPower(double power) {
        if (motor.getCurrentPosition() <= 0) {
            motor.setPower(Range.clip(power, 0, 1));
        } else if (motor.getCurrentPosition() >= maxRotation) {
            motor.setPower(Range.clip(power, -1, 0));
        } else {
            motor.setPower(power);
        }
    }

}
