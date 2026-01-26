package org.firstinspires.ftc.teamcode.decodeOpModes;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
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
import org.firstinspires.ftc.teamcode.decodeCommands.LimelightCommands;
import org.firstinspires.ftc.teamcode.decodeSubsystems.AutoShooter;
import org.firstinspires.ftc.teamcode.decodeSubsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.DischargeSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.decodeSubsystems.Motif;
import org.firstinspires.ftc.teamcode.decodeSubsystems.SavedValues;
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
    Motif motif = Motif.GPP;
    AutoShooter.TeamColor teamColor;
    ElapsedTime time;
    boolean endGame = false;

    @Override
    public void initialize() {
        time = new ElapsedTime();
        teamColor = SavedValues.teamColor;

        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        dischargeSubsystem = new DischargeSubsystem(hardwareMap);
        carouselSubsystem = new CarouselSubsystem(hardwareMap);

        mecanumDrive = new MecanumDrive(hardwareMap, SavedValues.position);

        limelightSubsystem = new LimelightSubsystem(hardwareMap, teamColor, mecanumDrive);

        driver = new GamepadEx(gamepad1);
        system = new GamepadEx(gamepad2);
        initButtons();

        mecanumDrive.setDefaultCommand(new MecanumCommands.Drive(mecanumDrive, () -> driver.getLeftY(), () -> driver.getLeftX(), () -> driver.getRightX(), () -> 1 - 0.5 * gamepad1.right_trigger, teamColor));
        dischargeSubsystem.setDefaultCommand(new DischargeCommands.AutomaticAiming(dischargeSubsystem, limelightSubsystem, mecanumDrive, carouselSubsystem, teamColor));
        driverA.whenPressed(new CommandGroups.StartIntake(intakeSubsystem, carouselSubsystem));
        driverB.whenPressed(new SequentialCommandGroup(
                new CommandGroups.PrepareShooting(intakeSubsystem, carouselSubsystem, mecanumDrive, teamColor),
                new CommandGroups.StartIntake(intakeSubsystem,carouselSubsystem).withTimeout(800)));
//        driverBack.whenPressed(new InstantCommand(() -> mecanumDrive.localizer.setPose(new Pose2d(mecanumDrive.localizer.getPose().position, Math.PI))));
        driverX.whenPressed(new SequentialCommandGroup(
                new CommandGroups.Shoot(intakeSubsystem, carouselSubsystem),
                new CommandGroups.StartIntake(intakeSubsystem, carouselSubsystem).withTimeout(800)));
        driverDPadLeft.whenPressed(() -> DischargeCommands.AutomaticAiming.aim = !DischargeCommands.AutomaticAiming.aim);
        driverLeftBumper.whenPressed(() -> IntakeCommands.IntakeState.resetCount = !IntakeCommands.IntakeState.resetCount);

        driverY.whenPressed(new CommandGroups.StartOuttake(intakeSubsystem,carouselSubsystem));
        driverRightBumper.whenPressed(new IntakeCommands.ClosedState(intakeSubsystem));
        mecanumDrive.lazyImu.get().resetYaw();
        systemDPadLeft.whenPressed(() -> DischargeCommands.AutomaticAiming.turretCorrection += 2);
        systemDPadRight.whenPressed(() -> DischargeCommands.AutomaticAiming.turretCorrection -= 2);
        systemDPadUp.whenPressed(() -> DischargeCommands.AutomaticAiming.rpmCorrection += 25);
        systemDPadDown.whenPressed(() -> DischargeCommands.AutomaticAiming.rpmCorrection -= 25);

//        driverDPadLeft.whenPressed(() -> DischargeCommands.AutomaticAiming.turretCorrection += 2);
//        driverDPadRight.whenPressed(() -> DischargeCommands.AutomaticAiming.turretCorrection -= 2);
//        driverDPadUp.whenPressed(() -> DischargeCommands.AutomaticAiming.rpmCorrection += 25);
//        driverDPadDown.whenPressed(() -> DischargeCommands.AutomaticAiming.rpmCorrection -= 25);

        systemA.whenPressed(() -> gamepad1.rumble(500));
//        systemLeftStick.whenPressed(new CarouselCommands.SafeSlide(carouselSubsystem,intakeSubsystem,0.3,0.8));
//        systemB.whenPressed(() -> carouselSubsystem.artifactsInGoal = (carouselSubsystem.artifactsInGoal + 1) % 3);
//        systemA.whenPressed(() -> carouselSubsystem.resetEncoders());
//        schedule(new LimelightCommands.KalmanFilter(limelightSubsystem,mecanumDrive,dischargeSubsystem::getTurretAngle, SavedValues.covariances.getX(),SavedValues.covariances.getY()));
    }

    @Override
    public void initialize_loop() {
        time.reset();
    }

    @Override
    public void run() {
        mecanumDrive.updatePoseEstimate();

        if(gamepad1.start && gamepad1.x){
            mecanumDrive.localizer.setPose(new Pose2d(mecanumDrive.localizer.getPose().position,0));
        }
        if(!DischargeCommands.AutomaticAiming.inRange && AutoShooter.canLaunch(mecanumDrive.localizer.getPose())){
            gamepad1.rumble(100);
        }
        if(!endGame && time.seconds() > 90){
            gamepad2.rumble(1000);
        }
        super.run();
        multipleTelemetry.addLine("Position");
        multipleTelemetry.addData("X", mecanumDrive.localizer.getPose().position.x);
        multipleTelemetry.addData("Y", mecanumDrive.localizer.getPose().position.y);
        multipleTelemetry.addData("heading", mecanumDrive.localizer.getPose().heading.toDouble() * 180 / Math.PI);
        multipleTelemetry.addLine("Carousel");
        multipleTelemetry.addData("current", carouselSubsystem.getCurrent());
        multipleTelemetry.addData("carouselAngle", carouselSubsystem.getAngle());
        multipleTelemetry.addData("carouselPosition", carouselSubsystem.getPosition());
        multipleTelemetry.addLine("Discharge");
        multipleTelemetry.addData("turretAngle", dischargeSubsystem.getTurretAngle());
        multipleTelemetry.addData("rpm", dischargeSubsystem.getRPM());
        multipleTelemetry.addData("flyWheelPower", dischargeSubsystem.flyWheelMotor.motorEx.getPower());
        multipleTelemetry.addData("correction",DischargeCommands.AutomaticAiming.turretCorrection);
        multipleTelemetry.update();
        SavedValues.position = mecanumDrive.localizer.getPose();
    }

    @Override
    public void end() {
//        carouselSubsystem.resetEncoders();
        limelightSubsystem.stopLimelight();
//        SavedValues.position = new Pose2d(63, 0, Math.PI);
//        SavedValues.carouselTicks = 0;
//        SavedValues.covariances = new Vector2d(0,0);

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
