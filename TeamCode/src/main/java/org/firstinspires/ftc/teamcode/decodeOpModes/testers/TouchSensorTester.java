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
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.actions.ActionOpMode;
import org.firstinspires.ftc.teamcode.decodeCommands.IntakeCommands;
import org.firstinspires.ftc.teamcode.decodeSubsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.needle.commands.MecanumCommands;

@TeleOp(group = "tests")
public class TouchSensorTester extends ActionOpMode {


    double lastTime, currentTime;
    boolean last = false;
    DigitalChannel left;
    IntakeSubsystem intakeSubsystem;
    int i = 0;
    GamepadEx gamepad;
    Button A,B,X,Y;


    @Override
    public void initialize() {
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        gamepad = new GamepadEx(gamepad1);
        A = new GamepadButton(gamepad, GamepadKeys.Button.A);
        Y = new GamepadButton(gamepad, GamepadKeys.Button.Y);
        X = new GamepadButton(gamepad, GamepadKeys.Button.X);
        B = new GamepadButton(gamepad, GamepadKeys.Button.B);;
        X.whenPressed(new IntakeCommands.IntakeState(intakeSubsystem));
        Y.whenPressed(new IntakeCommands.OutTakeState(intakeSubsystem));
        B.whenPressed(new IntakeCommands.ClosedState(intakeSubsystem));
    }

    @Override
    public void run() {
        super.run();

        multipleTelemetry.addData("count",IntakeCommands.IntakeState.count);
//        multipleTelemetry.addData("count",i);
        multipleTelemetry.update();
    }
}

