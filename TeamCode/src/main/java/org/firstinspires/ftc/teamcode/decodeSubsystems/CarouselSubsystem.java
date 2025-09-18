package org.firstinspires.ftc.teamcode.decodeSubsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class CarouselSubsystem extends SubsystemBase {
    private DcMotorEx carouselMotor;
    private Servo rampServo;

    public CarouselSubsystem(HardwareMap hm) {
        carouselMotor = hm.get(DcMotorEx.class, "carousel");
        carouselMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carouselMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rampServo = hm.servo.get("rampServo");
    }

    public void setSpinPower(double power) {
        carouselMotor.setPower(power);
    }

    public double getPosition() {
        return carouselMotor.getCurrentPosition();
    }

    public double getCurrent() {
        return carouselMotor.getCurrent(CurrentUnit.AMPS);
    }

    public void setRampServo(double pos) {
        rampServo.setPosition(pos);
    }

}
