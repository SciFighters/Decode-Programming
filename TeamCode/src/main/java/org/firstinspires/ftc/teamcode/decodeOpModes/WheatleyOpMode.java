package org.firstinspires.ftc.teamcode.decodeOpModes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.actions.ActionOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.decodeCommands.CarouselCommands;
import org.firstinspires.ftc.teamcode.decodeCommands.CommandGroups;
import org.firstinspires.ftc.teamcode.decodeCommands.DischargeCommands;
import org.firstinspires.ftc.teamcode.decodeCommands.IntakeCommands;
import org.firstinspires.ftc.teamcode.decodeSubsystems.AutoShooter;
import org.firstinspires.ftc.teamcode.decodeSubsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.needle.commands.MecanumCommands;

@TeleOp
public class WheatleyOpMode extends ActionOpMode {
    DischargeSubsystem dischargeSubsystem;
    IntakeSubsystem intakeSubsystem;
    CarouselSubsystem carouselSubsystem;
    MecanumDrive mecanumDrive;
    LimelightSubsystem limelightSubsystem;
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
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        dischargeSubsystem = new DischargeSubsystem(hardwareMap);
        carouselSubsystem = new CarouselSubsystem(hardwareMap);

        mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(new Vector2d(63, 0), Math.PI));

        limelightSubsystem = new LimelightSubsystem(hardwareMap, AutoShooter.TeamColor.RED, mecanumDrive);

        driver = new GamepadEx(gamepad1);
        system = new GamepadEx(gamepad2);
        initButtons();

        mecanumDrive.setDefaultCommand(new MecanumCommands.Drive(mecanumDrive, () -> driver.getLeftY(), () -> driver.getLeftX(), () -> driver.getRightX(), () -> 0.6 + 0.4 * driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)));
//        dischargeSubsystem.setDefaultCommand(new DischargeCommands.AutomaticAiming(dischargeSubsystem, limelightSubsystem, mecanumDrive, AutoShooter.TeamColor.RED));
        driverA.whenPressed(new IntakeCommands.IntakeState(intakeSubsystem));
        driverB.whenPressed(new CarouselCommands.MoveToPos(carouselSubsystem, carouselSubsystem.spinConversion));
        driverX.whenPressed(new SequentialCommandGroup(
                new IntakeCommands.TransferState(intakeSubsystem),
                new WaitCommand(2000),
                new CarouselCommands.Discharge(carouselSubsystem)
//                new IntakeCommands.TransferState(intakeSubsystem)
        ));
        driverRightStick.whenPressed(
                new ParallelCommandGroup(
                        new IntakeCommands.IntakeState(intakeSubsystem),
                        new CarouselCommands.MoveToPos(carouselSubsystem, carouselSubsystem.spinConversion * 1)
                ));
        driverY.whenPressed(new IntakeCommands.OutTakeState(intakeSubsystem));
        driverDPadDown.whenPressed(new DischargeCommands.setState(dischargeSubsystem, 0, 54));
        driverDPadUp.whenPressed(new IntakeCommands.ClosedState(intakeSubsystem));
        driverDPadLeft.whenPressed(new CarouselCommands.MoveToPos(carouselSubsystem, -carouselSubsystem.spinConversion * 1.1));
//        driverDPadRight.whenPressed(new CommandGroups.Shoot(dischargeSubsystem, intakeSubsystem, carouselSubsystem, 2800, 50));
        limelightSubsystem.startLimelight();
        mecanumDrive.lazyImu.get().resetYaw();
    }

    @Override
    public void run() {
        mecanumDrive.localizer.update();
        super.run();
//        multipleTelemetry.addData("err", ((launchAngle - dischargeSubsystem.getTurretAngle() + 180) % 360));
        multipleTelemetry.addData("turretAngle", dischargeSubsystem.getTurretAngle());
        multipleTelemetry.addData("launchAngle", DischargeCommands.AutomaticAiming.launchAngle);
        multipleTelemetry.addData("x", mecanumDrive.localizer.getPose().position.x);
        multipleTelemetry.addData("y", mecanumDrive.localizer.getPose().position.y);
//        multipleTelemetry.addData("angle", launchAngle);
        multipleTelemetry.addData("rpm", dischargeSubsystem.getRPM());
        multipleTelemetry.update();
    }

    @Override
    public void end() {
        limelightSubsystem.stopLimelight();
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
