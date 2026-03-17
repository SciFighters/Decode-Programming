package org.firstinspires.ftc.teamcode.decodeSubsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.jetbrains.annotations.Nullable;

public class CarouselSubsystem extends SubsystemBase {
    private final DcMotorEx carouselMotor;
    double tickPerRev = 384.5;//537.6


    public final double spinConversion = tickPerRev * 132.0 / 39.0 / 3.0; // for moving the motor a third of a spin
    public ColorSensor leftColorSensor, rightColorSensor, middleColorSensor;

    public int startingTicks = 0;

    public CarouselSubsystem(HardwareMap hm) {
        carouselMotor = hm.get(DcMotorEx.class, "carouselMotor");
//        carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftColorSensor = hm.get(ColorSensor.class, "leftColorSensor");
        rightColorSensor = hm.get(ColorSensor.class, "rightColorSensor");
        middleColorSensor = hm.get(ColorSensor.class, "middleColorSensor");
    }

    public void resetEncoders() {
        carouselMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carouselMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setSpinPower(double power) {
        carouselMotor.setPower(power);
    }

    public int getPosition() {
        return carouselMotor.getCurrentPosition() + startingTicks;
    }

    public void setTicks(int ticks) {
        startingTicks = ticks;
    }

    public double getAngle() {
        //weird algorithm for getting angle from 0 to 360
        return ((getPosition() / (spinConversion * 3) * 360) % 360 + 540) % 360;
    }
    public double getCarouselDistance(){
        return (getPosition() / (spinConversion * 3));
    }

    public double getCurrent() {
        return carouselMotor.getCurrent(CurrentUnit.AMPS);
    }

    public enum SensorColors {
        Purple,
        Green,
        Unknown
    }

    public Boolean isFull() {
        return (colorIdentifier(leftColorSensor) != SensorColors.Unknown &&
                colorIdentifier(rightColorSensor) != SensorColors.Unknown &&
                colorIdentifier(middleColorSensor) != SensorColors.Unknown);
    }

    public int getGreenPlacement() {//0 is first one
        if (colorIdentifier(rightColorSensor) == SensorColors.Green) {
            return 0;
        } else if (colorIdentifier(middleColorSensor) == SensorColors.Green) {
            return 1;
        } else if (colorIdentifier(leftColorSensor) == SensorColors.Green) {
            return 2;
        }
        return -1;
    }

    public static SensorColors colorIdentifier(ColorSensor colorSensor) {
        int green = colorSensor.green();
        int blue = colorSensor.blue();

        if (green + blue > 220) {
            if (green > blue) {
                return SensorColors.Green;
            }
            return SensorColors.Purple;
        }
        return SensorColors.Unknown;
    }
}