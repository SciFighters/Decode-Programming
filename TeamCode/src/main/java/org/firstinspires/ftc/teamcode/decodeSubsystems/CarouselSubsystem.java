package org.firstinspires.ftc.teamcode.decodeSubsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
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
    public final int spinConversion = 666; // for moving the motor about a third of a spin
    // should be changed once more info is provided

    private ColorSensor colorSlot1;
    private ColorSensor colorSlot2;
    private ColorSensor colorSlot3;
    public CarouselSubsystem(HardwareMap hm) {
        carouselMotor = hm.get(DcMotorEx.class, "carousel");
        carouselMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carouselMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rampServo = hm.servo.get("rampServo");

        colorSlot1 = hm.get(ColorSensor.class, "sensor1");
        colorSlot2 = hm.get(ColorSensor.class, "sensor2");
        colorSlot3 = hm.get(ColorSensor.class, "sensor3");
    }

    public void setSpinPower(double power) {
        carouselMotor.setPower(power);
    }

    public void moveToPosition(int ticks, double power) {
        carouselMotor.setTargetPosition(ticks);
        carouselMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        carouselMotor.setPower(power);
    }

    public boolean atTarget() {
        return !carouselMotor.isBusy();
    }

    public void stopMotor() {
        carouselMotor.setPower(0);
        carouselMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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

    public String colorIdentifier(ColorSensor sensor) {
        int red = sensor.red();
        int green = sensor.green();
        int blue = sensor.blue();
        if (green > 50) {
            return "green";
        } else if (blue > 50 && red > 50) {
            return "purple";
        }
        else return null;
    }
}
