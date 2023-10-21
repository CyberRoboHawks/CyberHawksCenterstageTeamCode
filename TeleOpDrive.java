package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;


@TeleOp(name = "TeleOpDrive", group = "TeleOp") // add this code
//@Disabled
public class TeleOpDrive extends LinearOpMode {
    static final double STANDARD_DRIVE_SPEED = .3;
    static final double TURBO_DRIVE_SPEED = .6;
    static final double STRAFE_SPEED = .4;
    private final ElapsedTime gametime = new ElapsedTime();
    HardwareMapping robot = new HardwareMapping();   // Use our hardware mapping
    Commands commands = new Commands();
    AprilTagProcessor tagProcessor = null;
    private final ElapsedTime runtime = new ElapsedTime();
    /*static final double LIFT_MAX_UP_POWER = .5;
    static final double LIFT_MAX_DOWN_POWER = .15;
    static final double LIFT_HOLD_POWER = .2;*/

    public void printRobotStatus() {
        telemetry.addData("hasArmMotors: ", robot.hasArmMotors);
        telemetry.addData("hasCamera: ", robot.hasCamera);
        telemetry.addData("hasDriveMotors: ", robot.hasDriveMotors);
        telemetry.addData("hasDroneServo: ", robot.hasDroneServo);
        telemetry.addData("hasGrabberDistance: ", robot.hasGrabberDistance);
        telemetry.addData("hasGrabberServo: ", robot.hasGrabberServo);
        telemetry.addData("hasGripperSlideServo: ", robot.hasGripperSlideServo);
        telemetry.addData("hasLinearActuatorMotor: ", robot.hasLinearActuatorMotor);
        telemetry.addData("hasPixelServo: ", robot.hasPixelServo);
        telemetry.addData("hasWristServo: ", robot.hasWristServo);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the hardware variables.
        robot.init(hardwareMap);
        commands.init(hardwareMap);
        if (robot.isRoboHawks) {
            commands.reverseMotorDirection();
        }
        double armPower;
        double strafePower;
        double forwardPower;
        double rotatePower;
        double offsetHeading = 0;
        boolean isReverse = false;
        boolean isGrabberOpen = false;
        boolean isWristUp = true;
        CenterStageEnums.Position grabberPosition = CenterStageEnums.Position.Up;

        if (robot.hasCamera) {
            tagProcessor = new AprilTagProcessor.Builder()
                    .setDrawCubeProjection(true)
                    .setDrawTagID(true)
                    .build();
            VisionPortal visionPortal = new VisionPortal.Builder()
                    .addProcessor(tagProcessor)
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .setCameraResolution(new Size(1920, 1080))
                    .build();
        }
        printRobotStatus();
        telemetry.addData("Status:", "Ready");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        gametime.reset();

        //robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Game Time: " + gametime);

            if (!isStopRequested() && robot.hasCamera && !gamepad1.a) {
                int targetId = 5;
                followTag(tagProcessor, CenterStageEnums.FollowDirection.Rotate, targetId);
                followTag(tagProcessor, CenterStageEnums.FollowDirection.Strafe, targetId);
                followTag(tagProcessor, CenterStageEnums.FollowDirection.Straight, targetId);
            }

            double drivePower = STANDARD_DRIVE_SPEED;  //1 is 100%, .5 is 50%

            //Driver controller ---------------------
            if (gamepad1 != null) {
                if (gamepad1.left_bumper && gamepad1.right_bumper) {
                    isReverse = commands.reverseMotorDirection();
                    sleep(200);
                }

                if (gamepad1.left_trigger > 0 && gamepad1.right_trigger > 0) {
                    telemetry.addData("triggers pressed", true);
                    telemetry.update();
                }

                // Turbo driving
                if (gamepad1.left_bumper) {
                    drivePower = TURBO_DRIVE_SPEED;  // change drive speed to the turbo speed variable
                }

                if (gamepad1.a && robot.hasPixelServo) {
                    commands.deliverPixel();
                }

                // mecanum driving
                strafePower = gamepad1.left_stick_x;
                forwardPower = -gamepad1.left_stick_y;
                rotatePower = gamepad1.right_stick_x;
                if (isReverse) {
                    rotatePower *= -1;
                }
                offsetHeading = 0;
                driveFieldCentric(strafePower, forwardPower, rotatePower, drivePower, offsetHeading);
            }

            //Co-Driver controller ---------------------
            if (gamepad2 != null) {
                if (commands.hasGripperSlideServo) {
                    if (gamepad2.dpad_left) {
                        robot.gripperSlideServo.setDirection(DcMotorSimple.Direction.FORWARD);
                        robot.gripperSlideServo.setPower(1);
                        sleep(100);
                    } else if (gamepad2.dpad_right) {
                        robot.gripperSlideServo.setDirection(DcMotorSimple.Direction.REVERSE);
                        robot.gripperSlideServo.setPower(1);
                        sleep(100);
                    }
                    robot.gripperSlideServo.setDirection(DcMotorSimple.Direction.REVERSE);
                    robot.gripperSlideServo.setPower(0);
                }

                if (gamepad2.a && robot.hasGrabberServo) {
                    if (isGrabberOpen) {
                        commands.servoSetPos(0.4);
                    } else {
                        commands.servoSetPos(0.6);
                    }
                    isGrabberOpen = !isGrabberOpen;
                    sleep(250);
                }

                if (gamepad2.y && robot.hasDroneServo) {
                    robot.droneServo.setPosition(1);
                    sleep(500);
                    robot.droneServo.setPosition(0);
                }

                if (gamepad2.x && robot.hasWristServo) {
                     double distanceCm = robot.grabberDistance.getDistance(DistanceUnit.CM);
                     if (distanceCm >10) {
                        robot.wristServo.setPosition(.13);
                    } else {
                        robot.wristServo.setPosition(0.7);
                    }
                    sleep(200);
                }
            }

            armPower = -gamepad2.left_stick_y;
            if (armPower !=0){
                if (armPower > 0){
                    setArmPosition(CenterStageEnums.ArmDirection.Up, 2);
                } else{
                    setArmPosition(CenterStageEnums.ArmDirection.Down, 2);
                }
            }
        }
        telemetry.update();
    }

    private void setArmPosition(CenterStageEnums.ArmDirection armDirection, double timeout) {
        if (!robot.hasArmMotors)
            return;

        double position =0;
        double power = 0;
        boolean isArmMoving = true;

        runtime.reset();

        while (isArmMoving && (runtime.seconds() < timeout)) {
            position = robot.armMotorRight.getCurrentPosition();// + robot.armMotorLeft.getCurrentPosition()) / 2.0;
            telemetry.addData("direction:", armDirection);
            // 0 - 350
            if (armDirection == CenterStageEnums.ArmDirection.Up) {
                if (position < 120) {
                    power = .10;
                } else if (position < 200) {
                    power = .05;
                } else if (position < 250) {
                    power = -.15;
                } else {
                    power = 0;
                    isArmMoving = false;
                }
            } else if (armDirection == CenterStageEnums.ArmDirection.Down) {
                if (position > 225) {
                    power = -.15;
                } else if (position > 120) {
                    power = .15;
                } else if (position > 25) {
                    power = .075;
                } else {
                    power = 0;
                    isArmMoving = false;
                }
            }
            telemetry.addData("position:", position);
            telemetry.addData("arm power:", power);
            telemetry.update();
            robot.armMotorRight.setPower(power);
            robot.armMotorLeft.setPower(power);
            if (power == 0){
                isArmMoving = false;
            }
        }
        position = robot.armMotorRight.getCurrentPosition();// + robot.armMotorLeft.getCurrentPosition()) / 2.0;

        telemetry.addData("position:", position);
        telemetry.addData("arm power:", power);
        telemetry.update();
    }

    private void followTag(AprilTagProcessor tagProcessor, CenterStageEnums.FollowDirection direction, int targetId) {
        if (tagProcessor.getDetections().size() > 0) {
            AprilTagProcessor myAprilTagProcessor;
            List<AprilTagDetection> myAprilTagDetections;  // list of all detections
            AprilTagDetection tag = null;
            myAprilTagDetections = tagProcessor.getDetections();
            for (AprilTagDetection myAprilTagDetection : myAprilTagDetections) {
                if (myAprilTagDetection.metadata != null) {  // This check for non-null Metadata is not needed for reading only ID code.
                    if (myAprilTagDetection.id == targetId) {
                        tag = myAprilTagDetection;
                        break;
                    }
                }
            }

            if (tag == null) {
                return;
            }
            telemetry.addData("ID", tag.id);
            telemetry.addData("strafe yaw", tag.ftcPose.yaw);
            telemetry.addData("distance range", tag.ftcPose.range);
            telemetry.addData("rotate angle", tag.ftcPose.bearing);
            telemetry.update();
            double speed = .3;
            double straightspeed = .3;
            double targetDistance = 12;
            double heading = tag.ftcPose.bearing;
            double yaw = tag.ftcPose.yaw;//tag.ftcPose.x;
            double distance = tag.ftcPose.range;//tag.ftcPose.y;
            double cameraOffset = 0;
            //boolean currentside = true; // true is right size, false is left side

            double angleGain = .3;
            double strafeGain = .2;
            double straightGain = .4;
            if (distance > 30) {
                angleGain = .4;
                strafeGain = .3;
                straightGain = .6;
            }

            if (direction == CenterStageEnums.FollowDirection.Straight) {
                if (distance > targetDistance) {
                    commands.driveForward(straightspeed, (abs(distance) - targetDistance) * straightGain, 3, telemetry);
                }
                if (heading > 20) {
                    commands.driveBackwards(straightspeed, 3, 3, telemetry);
                    commands.strafeLeft(speed, 2, 2);
                }
                if (heading < -20) {
                    commands.driveBackwards(straightspeed, 3, 3, telemetry);
                    commands.strafeRight(speed, 2, 2);
                }
            }

            if (direction == CenterStageEnums.FollowDirection.Rotate) {
                double targetAngle = getAngle() + heading * angleGain;
                if (heading > 4) {
                    commands.spinLeft(speed, targetAngle, 1);
                } else if (heading < -4) {
                    commands.spinRight(speed, targetAngle, 1);
                }
            }

            if (direction == CenterStageEnums.FollowDirection.Strafe) {
                double strafeTarget = (abs(yaw) * strafeGain - cameraOffset);
                if (yaw > 4) {
                    commands.strafeRight(speed, strafeTarget, 3);
                } else if (yaw < -4) {
                    commands.strafeLeft(speed, strafeTarget, 3);
                }
            }
            sleep(50);
        }
    }

    private void driveFieldCentric(double strafeSpeed, double forwardSpeed, double rotate, double drivePower, double heading) {
        if (!robot.hasDriveMotors)
            return;

        //finds just how much power to give the robot based on how much x and y given by gamepad
        //range.clip helps us keep our power within positive 1
        // also helps set maximum possible value of 1/sqrt(2) for x and y controls if at a 45 degree angle (which yields greatest possible value for y+x)
        double power = Range.clip(Math.hypot(strafeSpeed, forwardSpeed), 0, 1);

        //the inverse tangent of opposite/adjacent gives us our gamepad degree
        double theta = Math.atan2(forwardSpeed, strafeSpeed);
        double movementDegree = theta - Math.toRadians(heading);

        // Calculate the adjusted degrees for x and y movement
        double yOffset = Math.sin(movementDegree - Math.PI / 4);
        double xOffset = Math.cos(movementDegree - Math.PI / 4);

        double maxOffset = Math.max(abs(yOffset), abs(xOffset));

        // normalize the x/y offset and apply control stick power
        yOffset = power * yOffset / maxOffset;
        xOffset = power * xOffset / maxOffset;

        if (Math.abs(strafeSpeed) > .2) {
            drivePower = STRAFE_SPEED;
        }

        double leftFrontPower = (xOffset + rotate) * drivePower;
        double leftBackPower = (yOffset + rotate) * drivePower;
        double rightFrontPower = (yOffset - rotate) * drivePower;
        double rightBackPower = (xOffset - rotate) * drivePower;

        // Set the power on each motor
        robot.leftFrontMotor.setPower(leftFrontPower);
        robot.leftBackMotor.setPower(leftBackPower);
        robot.rightFrontMotor.setPower(rightFrontPower);
        robot.rightBackMotor.setPower(rightBackPower);
    }

    //allows us to quickly get our z angle
    private double getAngle() {
        if (robot.isRoboHawks) {
            return robot.imuRoboHawks.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        } else {
            return robot.imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        }
    }
}