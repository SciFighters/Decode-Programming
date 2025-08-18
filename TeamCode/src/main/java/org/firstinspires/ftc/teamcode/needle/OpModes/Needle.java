package org.firstinspires.ftc.teamcode.needle.OpModes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.actions.ActionCommand;
import org.firstinspires.ftc.teamcode.actions.ActionOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.needle.commands.ArmAxisCommands;
import org.firstinspires.ftc.teamcode.needle.commands.Groups;
import org.firstinspires.ftc.teamcode.needle.commands.MecanumCommands;
import org.firstinspires.ftc.teamcode.needle.commands.TelescopicArmCommands;
import org.firstinspires.ftc.teamcode.needle.subsystems.ArmAxisSubsystem;
import org.firstinspires.ftc.teamcode.needle.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.needle.subsystems.TelescopicArmSubsystem;

import java.util.HashSet;
import java.util.Set;

@TeleOp
public class Needle extends ActionOpMode {
    ArmAxisSubsystem armAxisSubsystem;
    TelescopicArmSubsystem telescopicArmSubsystem;
    ClawSubsystem clawSubsystem;
    MecanumDrive mecanumDrive;
    GamepadEx driver, system;
    Button driverA, systemA;
    Button driverB, systemB;
    Button driverY, systemY;
    Button driverX, systemX;
    Button driverDPadDown, systemDPadDown;
    Button driverDPadUp, systemDPadUp;
    Button driverDPadRight, systemDPadRight;
    Button driverDPadLeft, systemDPadLeft;
    Button driverRightBumper, systemRightBumper;
    Button driverLeftBumper, systemLeftBumper;
    Button driverStart, systemStart;
    Button driverBack, systemBack;
    Button driverLeftStick, systemLeftStick, driverRightStick, systemRightStick;
    @Override
    public void initialize() {
        armAxisSubsystem = new ArmAxisSubsystem(hardwareMap);
        telescopicArmSubsystem = new TelescopicArmSubsystem(hardwareMap);
        mecanumDrive = new MecanumDrive(hardwareMap,new Pose2d(new Vector2d(0,-64),Math.PI/2));
//        TrajectoryActionBuilder turnBasket = mecanumDrive.actionBuilder(mecanumDrive.localizer.getPose()).turnTo(Math.PI/4);
//        Set<Subsystem> requirements = new HashSet<Subsystem>();
//        requirements.add(mecanumDrive);

        driver = new GamepadEx(gamepad1);
        system = new GamepadEx(gamepad2);
        initButtons();

        armAxisSubsystem.setDefaultCommand(new ArmAxisCommands.AxisManual(armAxisSubsystem, system::getRightY));
        telescopicArmSubsystem.setDefaultCommand(new TelescopicArmCommands.ExtensionManual(telescopicArmSubsystem,
                system::getLeftY));//() -> gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)
        mecanumDrive.setDefaultCommand(new MecanumCommands.Drive(mecanumDrive,() -> driver.getLeftX(),() -> driver.getLeftY(),() -> driver.getRightX(),() -> 0.6 + 0.4 * driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)));
        driverB.whenPressed(new Runnable() {
            @Override
            public void run() {
                TrajectoryActionBuilder turnBasket = mecanumDrive.actionBuilder(mecanumDrive.localizer.getPose()).turnTo(Math.PI/4);
                Set<Subsystem> requirements = new HashSet<Subsystem>();
                requirements.add(mecanumDrive);
                CommandScheduler.getInstance().schedule(new ActionCommand(turnBasket.build(),requirements));
            }
        });
        driverY.whenPressed(new Runnable() {
            @Override
            public void run() {
                TrajectoryActionBuilder gotoBasket = mecanumDrive.actionBuilder(mecanumDrive.localizer.getPose())
                        .setTangent(Math.PI)
                        .splineToLinearHeading(new Pose2d(-50,-50,Math.PI / 4),Math.PI);
                Set<Subsystem> requirements = new HashSet<Subsystem>();
                requirements.add(mecanumDrive);
                CommandScheduler.getInstance().schedule(new ParallelCommandGroup(new ActionCommand(gotoBasket.build(),requirements),
                        new SequentialCommandGroup(new WaitCommand(500),new Groups.GoToBasketCmd(telescopicArmSubsystem, armAxisSubsystem, clawSubsystem))));
            }
        });
        systemA.whenPressed(new Groups.GoToBasketCmd(telescopicArmSubsystem,armAxisSubsystem,clawSubsystem));
        systemB.whenPressed(new Groups.GoHome(telescopicArmSubsystem,armAxisSubsystem,clawSubsystem));

    }

    @Override
    public void run() {

        super.run();
        mecanumDrive.updatePoseEstimate();
//        multipleTelemetry.addData("heading deg",Math.toDegrees(mecanumDrive.localizer.getPose().heading.toDouble()));
//        multipleTelemetry.addData("armCm",telescopicArmSubsystem.getPosition());
//        multipleTelemetry.addData("telescopicCurrent",telescopicArmSubsystem.getCurrent());
//        multipleTelemetry.addData("axisCurrent",armAxisSubsystem.getCurrent());
//        multipleTelemetry.addData("armAngle",armAxisSubsystem.getAngle());
//        multipleTelemetry.update();
    }

    public void initButtons() {
        driverA = new GamepadButton(driver, GamepadKeys.Button.A);
        driverB = new GamepadButton(driver, GamepadKeys.Button.B);
        driverY = new GamepadButton(driver, GamepadKeys.Button.Y);
        driverX = new GamepadButton(driver, GamepadKeys.Button.X);
        driverDPadDown = new GamepadButton(driver, GamepadKeys.Button.DPAD_DOWN);
        driverDPadUp = new GamepadButton(driver, GamepadKeys.Button.DPAD_UP);
        driverDPadRight = new GamepadButton(driver, GamepadKeys.Button.DPAD_RIGHT);
        driverDPadLeft = new GamepadButton(driver, GamepadKeys.Button.DPAD_LEFT);
        driverRightBumper = new GamepadButton(driver, GamepadKeys.Button.RIGHT_BUMPER);
        driverLeftBumper = new GamepadButton(driver, GamepadKeys.Button.LEFT_BUMPER);
        driverStart = new GamepadButton(driver, GamepadKeys.Button.START);
        driverLeftStick = new GamepadButton(driver, GamepadKeys.Button.LEFT_STICK_BUTTON);
        driverRightStick = new GamepadButton(driver, GamepadKeys.Button.RIGHT_STICK_BUTTON);
        driverBack = new GamepadButton(driver, GamepadKeys.Button.BACK);
        systemA = new GamepadButton(system, GamepadKeys.Button.A);
        systemB = new GamepadButton(system, GamepadKeys.Button.B);
        systemY = new GamepadButton(system, GamepadKeys.Button.Y);
        systemX = new GamepadButton(system, GamepadKeys.Button.X);
        systemDPadDown = new GamepadButton(system, GamepadKeys.Button.DPAD_DOWN);
        systemDPadUp = new GamepadButton(system, GamepadKeys.Button.DPAD_UP);
        systemDPadRight = new GamepadButton(system, GamepadKeys.Button.DPAD_RIGHT);
        systemDPadLeft = new GamepadButton(system, GamepadKeys.Button.DPAD_LEFT);
        systemRightBumper = new GamepadButton(system, GamepadKeys.Button.RIGHT_BUMPER);
        systemLeftBumper = new GamepadButton(system, GamepadKeys.Button.LEFT_BUMPER);
        systemStart = new GamepadButton(system, GamepadKeys.Button.START);
        systemLeftStick = new GamepadButton(system, GamepadKeys.Button.LEFT_STICK_BUTTON);
        systemRightStick = new GamepadButton(system, GamepadKeys.Button.RIGHT_STICK_BUTTON);
        systemBack = new GamepadButton(system, GamepadKeys.Button.BACK);
    }
}
