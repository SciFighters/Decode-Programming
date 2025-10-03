package org.firstinspires.ftc.teamcode.decodeSubsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class CarouselSubsystem extends SubsystemBase {
    private DcMotorEx carouselMotor;
    public final int spinConversion = 666; // for moving the motor about a third of a spin
    // should be changed once more info is provided
    public ColorSensor colorSensor;

    public CarouselSubsystem(HardwareMap hm) {
        carouselMotor = hm.get(DcMotorEx.class, "carousel");
        carouselMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carouselMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        colorSensor = hm.get(ColorSensor.class, "colorSensor");
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

    public enum SensorColors {
        Purple,
        Green,
        Unknown
    }

    public SensorColors colorIdentifier(ColorSensor sensor) {
        int red = sensor.red();
        int green = sensor.green();
        int blue = sensor.blue();

        if (red > green &&  blue > green) {
            return SensorColors.Purple;

        } else if (green > red && green > blue) {
            return SensorColors.Green;
        }
        return SensorColors.Unknown;
    }
}