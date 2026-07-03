package org.firstinspires.ftc.teamcode.decodeSubsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.SubsystemBase;


public class RampSubsystem extends SubsystemBase {

    private final Servo rampServo;

    public RampSubsystem(HardwareMap hm) {
        rampServo = hm.get(Servo.class, "rampServo");
    }


    public void setRampDegree(double rampDegree) {
        double pos = (60 - rampDegree) / 30.0;
        rampServo.setPosition(Math.min(pos * 0.6 - 0.2, 0.6));
    }
}
