package org.firstinspires.ftc.teamcode.needle.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class TelescopicArmSubsystem extends SubsystemBase {
    private DcMotorEx axisMotor, extensionMotor;
    public final double ticksPerCm = 8.4;//change later
    private final double maxArmLength = 1000;

    public TelescopicArmSubsystem(HardwareMap hm){
        axisMotor = hm.get(DcMotorEx.class, "axisMotor");
        extensionMotor = hm.get(DcMotorEx.class, "extensionMotor");
        axisMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        axisMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        axisMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void setArmPower(double power){
        if (extensionMotor.getCurrentPosition() <= 0) {
            extensionMotor.setPower(Range.clip(power, 0, 1));
        } else if (extensionMotor.getCurrentPosition() >= maxArmLength) {
            extensionMotor.setPower(Range.clip(power, -1, 0));
        } else {
            extensionMotor.setPower(power);
        }
    }
    public int getPosition(){
        return extensionMotor.getCurrentPosition();
    }
    public double getCurrent(){
        return extensionMotor.getCurrent(CurrentUnit.AMPS);
    }


}
