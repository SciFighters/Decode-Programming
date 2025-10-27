package org.firstinspires.ftc.teamcode.decodeSubsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class DischargeSubsystem extends SubsystemBase {

    private final DcMotorEx flyWheelMotor, turretMotor;
    private final Servo rampServo;


    public DischargeSubsystem(HardwareMap hm) {
        flyWheelMotor = hm.get(DcMotorEx.class, "flyWheelMotor");
        turretMotor = hm.get(DcMotorEx.class, "turretMotor");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rampServo = hm.get(Servo.class, "rampServo");
    }

    public void setFlyWheelPower(double flyWheelPower) {
        flyWheelMotor.setPower(flyWheelPower);
    }

    public void setTurretPower(double turretPower) {
        turretMotor.setPower(turretPower);
    }
    public double getTurretPosition(){
        return turretMotor.getCurrentPosition();
    }

    public void setRampDegree(double rampDegree) {
        rampServo.setPosition(rampDegree/90);
    }

}
