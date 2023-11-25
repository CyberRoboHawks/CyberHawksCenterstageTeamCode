package org.firstinspires.ftc.teamcode;

import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import static org.firstinspires.ftc.teamcode.CenterStageEnums.Position.Down;
import static org.firstinspires.ftc.teamcode.CenterStageEnums.Position.Up;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.CenterStageEnums.Position;

@TeleOp(name = "TeleOpDrive", group = "TeleOp") // add this code
//@Disabled
public class TeleOpDrive extends LinearOpMode {
    static final double STANDARD_DRIVE_SPEED = .4;
    static final double TURBO_DRIVE_SPEED = .9;

    private final ElapsedTime gametime = new ElapsedTime();
    HardwareMapping robot = new HardwareMapping();   // Use our hardware mapping
    Commands commands = new Commands(telemetry);
    boolean isLightsDisabled = false;
    boolean isReverse = false;
    boolean isLiftInitiated = false;
    boolean showDebug = false;
    double armPower;
    double strafePower;
    double forwardPower;
    double rotatePower;
    double offsetHeading = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the hardware variables.
        robot.init(hardwareMap, true);
        commands.init(hardwareMap, true);

        telemetry.addData("Status:", "Ready");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        gametime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (robot.hasDroneSecureServo) {
                robot.droneSecureServo.setPosition(Commands.DRONE_SECURE);
            }
            if (robot.hasArmDownSensor) {
                if (robot.armDownSensor.isPressed()) {
                    robot.armPosition = Down;
                    if (robot.armMotorRight.getCurrentPosition() != 0) {
                        commands.stopAndRestMotorEncoders(robot.armMotorRight);
                        commands.stopAndRestMotorEncoders(robot.armMotorLeft);
                    }
                } else {
                    robot.armPosition = Up;
                }
            }
            setLedColors();

            double drivePower = STANDARD_DRIVE_SPEED;  //1 is 100%, .5 is 50%

            //Driver controller ---------------------
            if (gamepad1 != null) {
                if (gamepad1.guide) toggleDebugMessages();
                if (gamepad1.back) toggleLeds();

                if (gamepad1.dpad_up && gamepad1.y) {
                    isReverse = commands.reverseDriveMotorDirection();
                    sleep(250);
                }

                if (gamepad1.a && robot.armPosition == Position.Up) {
                    commands.approachBackdrop(3, 2);
                    sleep(250);
                }

                // Turbo driving
                if (gamepad1.left_bumper || gamepad1.right_bumper) {
                    drivePower = TURBO_DRIVE_SPEED;  // change drive speed to the turbo speed variable
                }

                // mecanum driving
                strafePower = gamepad1.left_stick_x;
                forwardPower = -gamepad1.left_stick_y;
                rotatePower = gamepad1.right_stick_x;
                if (isReverse) {
                    rotatePower *= -1;
                }
                offsetHeading = 0;
                commands.driveMecanum(strafePower, forwardPower, rotatePower, drivePower, offsetHeading);
            }

            //Co-Driver controller ---------------------
            if (gamepad2 != null) {
                if (gamepad2.guide) toggleDebugMessages();
                if (gamepad2.back) toggleLeds();

                if (gamepad2.a && robot.hasGrabberServo) {
                    if (robot.grabberServo.getPosition() < robot.GRABBER_CLOSED) {
                        commands.setGrabberPosition(robot.GRABBER_CLOSED);
                    } else {
                        commands.setGrabberPosition(robot.GRABBER_OPEN);
                    }
                    sleep(250);
                }

                // prevent early launch with gameTimer check for endgame
                if (gamepad2.y && robot.hasDroneServo && (gametime.seconds() > 90 || gamepad2.right_bumper)) {
                    commands.launchDrone();
                    sleep(250);
                }

                if (gamepad2.x && robot.hasWristServo) {
                    if (robot.hasArmDownSensor && robot.armDownSensor.isPressed())
                        commands.reverseWristPosition();
                    sleep(250);
                }

                armPower = -gamepad2.left_stick_y;
                if (armPower != 0) {
                    if (armPower > .1 && !isLiftInitiated) {
                        commands.setArmPosition(CenterStageEnums.ArmDirection.Up, 3);
                    } else if (armPower < -.1) {
                        robot.wristServo.setPosition(robot.WRIST_UP);
                        commands.setArmPosition(CenterStageEnums.ArmDirection.Down, 3);
                    }

                    sleep(250);
                }

                if (robot.hasLinearActuatorMotor) {
                    if (gametime.seconds() >= 90 && robot.armPosition == Down) {
                        if (gamepad2.right_stick_y != 0) {
                            isLiftInitiated = true;
                            commands.moveLinearActuator(-gamepad2.right_stick_y, false);
                        }
                    }

                    // Allow override of linear actuator limits
                    if (gamepad2.right_stick_y != 0 && gamepad2.left_bumper) {
                        commands.moveLinearActuator(-gamepad2.right_stick_y, true);
                    }
                }
            }

            if (robot.hasLinearActuatorMotor) {
                if (!robot.linearActuatorMotor.isBusy())
                    robot.linearActuatorMotor.setPower(0);
            }

            if (showDebug) commands.printRobotStatus();
            telemetry.addData("Gametime", gametime.seconds());
            telemetry.addData("Drive Reversed", isReverse);
            telemetry.addData("r pos", robot.armMotorRight.getCurrentPosition());
            telemetry.addData("l pos", robot.armMotorLeft.getCurrentPosition());
            telemetry.addData("linear", robot.linearActuatorMotor.getCurrentPosition());
            telemetry.addData("distance", robot.grabberDistance.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }

    private void toggleLeds() {
        isLightsDisabled = !isLightsDisabled;
        sleep(250);
    }

    private void toggleDebugMessages() {
        showDebug = !showDebug;
        sleep(250);
    }

    private void setLedColors() {
        int GRABBER_TARGET_DISTANCE = 4;
        int ENDGAME = 90;
        int GAMEOVER = 120;

        if (robot.hasBlinkin) {
            if (isLightsDisabled) {
                robot.blinkinLedDriver.setPattern(BlinkinPattern.BLACK);
            } else if (robot.armPosition == Position.Up
                    && robot.hasGrabberDistance
                    && robot.grabberDistance.getDistance(DistanceUnit.CM) <= GRABBER_TARGET_DISTANCE) {
                robot.blinkinLedDriver.setPattern(BlinkinPattern.GREEN);
            } else if (gametime.seconds() >= ENDGAME && gametime.seconds() < ENDGAME + 20) {
                robot.blinkinLedDriver.setPattern(BlinkinPattern.BREATH_RED);
            } else if (gametime.seconds() >= ENDGAME + 20 && gametime.seconds() < GAMEOVER) {
                robot.blinkinLedDriver.setPattern(BlinkinPattern.HEARTBEAT_RED);
            } else if (gametime.seconds() > GAMEOVER) {
                robot.blinkinLedDriver.setPattern(BlinkinPattern.RED);
            } else if (isReverse) {
                robot.blinkinLedDriver.setPattern(BlinkinPattern.RAINBOW_FOREST_PALETTE);
            } else {
                robot.blinkinLedDriver.setPattern(BlinkinPattern.RAINBOW_OCEAN_PALETTE);
            }
        }
    }
}