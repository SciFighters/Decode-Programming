package org.firstinspires.ftc.teamcode.needle.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class TelescopicArmSubsystem extends SubsystemBase {
    private DcMotorEx extensionMotor;
    public final double ticksPerCm = 39.487962963;//change later
    private final int maxArmLength = 78, minArmLength = 30;
    public final double basketPos = 60, intakePos = 36;//tested frfr

    public TelescopicArmSubsystem(HardwareMap hm){
        extensionMotor = hm.get(DcMotorEx.class, "extensionMotor");
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void setArmPower(double power){
        if (getPosition() <= 0) {
            extensionMotor.setPower(Range.clip(power, 0, 1));
        } else if (getPosition() >= maxArmLength) {
            extensionMotor.setPower(Range.clip(power, -1, 0));
        } else {
            extensionMotor.setPower(power);
        }
    }
    public double getPosition(){
        return extensionMotor.getCurrentPosition() / ticksPerCm + minArmLength;
    }

    public double getCurrent(){
        return extensionMotor.getCurrent(CurrentUnit.AMPS);
    }
}
