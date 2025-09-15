package org.firstinspires.ftc.teamcode.needle.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawSubsystem extends SubsystemBase {
    public Servo axisServo, clampServo;
    public double intakePos = 0.5, dischargePos; //for axis
    public double releasePos, holdPos; //for clamp
    public final double basketTilt = 0.8;
    public ClawSubsystem(HardwareMap hm){
        axisServo = hm.servo.get("axisServo");
        clampServo = hm.servo.get("clampServo");
    }

    public void setAxisServo(double pos){
        axisServo.setPosition(pos);
    }

    public void setClampServo(double pos){
        clampServo.setPosition(pos);
    }
}
