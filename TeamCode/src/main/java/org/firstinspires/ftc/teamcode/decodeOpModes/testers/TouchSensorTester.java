package org.firstinspires.ftc.teamcode.decodeOpModes.testers;


import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.lynx.LynxController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.actions.ActionOpMode;
import org.firstinspires.ftc.teamcode.decodeCommands.IntakeCommands;
import org.firstinspires.ftc.teamcode.decodeSubsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.needle.commands.MecanumCommands;

@TeleOp(group = "tests")
public class TouchSensorTester extends ActionOpMode {

    ElapsedTime time;
    double lastTime, currentTime;
    boolean last = false;
    DigitalChannel touchSensor;
    int i = 0;


    @Override
    public void initialize() {
        touchSensor = hardwareMap.get(DigitalChannel.class,"touchSensor");
        time = new ElapsedTime();
        currentTime = time.seconds();
        lastTime = currentTime;
    }

    @Override
    public void run() {
        boolean current = !touchSensor.getState();
        if(current && !last){
          i += 1;
        }
        multipleTelemetry.addData("pressed",touchSensor.getState());
        currentTime = time.seconds();
        multipleTelemetry.addData("frameRate",1/(currentTime - lastTime));
        multipleTelemetry.addData("count",i);
        multipleTelemetry.update();
        lastTime = currentTime;
        last = current;
    }
}

