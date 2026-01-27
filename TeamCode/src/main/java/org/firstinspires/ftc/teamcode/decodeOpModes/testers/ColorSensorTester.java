package org.firstinspires.ftc.teamcode.decodeOpModes.testers;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.actions.ActionOpMode;
import org.firstinspires.ftc.teamcode.decodeSubsystems.CarouselSubsystem;

@TeleOp(group = "tests")
public class ColorSensorTester extends ActionOpMode {
    ColorSensor left, right, middle;

    @Override
    public void initialize() {
        left = hardwareMap.get(ColorSensor.class,"leftColorSensor");
        right = hardwareMap.get(ColorSensor.class,"rightColorSensor");
        middle = hardwareMap.get(ColorSensor.class,"middleColorSensor");
    }

    @Override
    public void run() {
        multipleTelemetry.addLine("left");

        multipleTelemetry.addData("left", CarouselSubsystem.colorIdentifier(left));
        multipleTelemetry.addData("leftRed",left.red());
        multipleTelemetry.addData("leftBlue",left.blue());
        multipleTelemetry.addData("leftGreen",left.green());

        multipleTelemetry.addLine("right");

        multipleTelemetry.addData("right", CarouselSubsystem.colorIdentifier(right));
        multipleTelemetry.addData("rightRed",right.red());
        multipleTelemetry.addData("rightBlue",right.blue());
        multipleTelemetry.addData("rightGreen",right.green());

        multipleTelemetry.addLine("middle");

        multipleTelemetry.addData("middle", CarouselSubsystem.colorIdentifier(middle));
        multipleTelemetry.addData("middleRed",middle.red());
        multipleTelemetry.addData("middleBlue",middle.blue());
        multipleTelemetry.addData("middleGreen",middle.green());
        multipleTelemetry.update();
    }
}
