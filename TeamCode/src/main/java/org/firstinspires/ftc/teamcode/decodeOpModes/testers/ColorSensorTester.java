package org.firstinspires.ftc.teamcode.decodeOpModes.testers;

import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.actions.ActionOpMode;

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
        multipleTelemetry.addData("leftRed",left.green());
        multipleTelemetry.addData("leftBlue",left.blue());
        multipleTelemetry.addData("leftGreen",left.red());
        multipleTelemetry.addData("rightRed",right.green());
        multipleTelemetry.addData("rightBlue",right.blue());
        multipleTelemetry.addData("rightGreen",right.red());
        multipleTelemetry.addData("middleRed",middle.green());
        multipleTelemetry.addData("middleBlue",middle.blue());
        multipleTelemetry.addData("middleGreen",middle.red());
        multipleTelemetry.update();
    }
}
