package org.firstinspires.ftc.teamcode.decodeSubsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class CarouselSubsystem extends SubsystemBase {
    private final DcMotorEx carouselMotor;
    double tickPerRev = 383.6;
    // accurate amount of tick per revolution
    public final double spinConversion = tickPerRev * 5.0 / 3.0; // for moving the motor about a third of a spin
    // calculation for a third of a spin knowing the amount of ticks per revolution
    public ColorSensor colorSensor;

    public CarouselSubsystem(HardwareMap hm) {
        carouselMotor = hm.get(DcMotorEx.class, "carousel");
        carouselMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carouselMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        colorSensor = hm.get(ColorSensor.class, "colorSensor");
    }

    public void setSpinPower(double power) {
        carouselMotor.setPower(power);
    }

    public double getPosition() {
        return carouselMotor.getCurrentPosition();
    }

    public double getAngle() {
        //weird algorithm for getting angle from 0 to 360
        return ((carouselMotor.getCurrentPosition() / spinConversion * 3) % 360 + 360) % 360;
    }

    public double getCurrent() {
        return carouselMotor.getCurrent(CurrentUnit.AMPS);
    }

    public enum SensorColors {
        Purple,
        Green,
        Unknown
    }

    public SensorColors colorIdentifier() {
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();

        if (red > green && blue > green) {
            return SensorColors.Purple;

        } else if (green > red && green > blue) {
            return SensorColors.Green;
        }
        return SensorColors.Unknown;
    }
}