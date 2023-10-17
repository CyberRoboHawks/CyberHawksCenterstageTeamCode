package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


@TeleOp(name = "TeleOpDrive", group = "TeleOp") // add this code
//@Disabled
public class TeleOpDrive extends LinearOpMode {
    static final double STANDARD_DRIVE_SPEED = .3;
    static final double TURBO_DRIVE_SPEED = .6;
    static final double STRAFE_SPEED = .4;
    HardwareMapping robot = new HardwareMapping();   // Use our hardware mapping
    Commands commands = new Commands();
    AprilTagProcessor tagProcessor = null;
    /*static final double LIFT_MAX_UP_POWER = .5;
    static final double LIFT_MAX_DOWN_POWER = .15;
    static final double LIFT_HOLD_POWER = .2;*/

    public void printRobotStatus() {
        telemetry.addData("hasArmMotors: ", robot.hasArmMotors);
        telemetry.addData("hasCamera: ", robot.hasCamera);
        telemetry.addData("hasDriveMotors: ", robot.hasDriveMotors);
        telemetry.addData("hasDroneServo: ", robot.hasDroneServo);
        telemetry.addData("hasGrabberServo: ", robot.hasGrabberServo);
        telemetry.addData("hasLinearActuatorMotor: ", robot.hasLinearActuatorMotor);
        telemetry.addData("hasPixelServo: ", robot.hasPixelServo);
        telemetry.addData("hasWristServo: ", robot.hasWristServo);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the hardware variables.
        robot.init(hardwareMap);
        commands.init(hardwareMap);

        double armPower;
        double strafePower;
        double forwardPower;
        double rotatePower;
        double offsetHeading = 0;
        boolean isReverse = false;
        boolean isGrabberOpen = false;
        CenterStageEnums.Position grabberPosition = CenterStageEnums.Position.Up;
        double liftPower = 0;

        if (robot.hasCamera) {
            tagProcessor = new AprilTagProcessor.Builder()
                    .setDrawCubeProjection(true)
                    .setDrawTagID(true)
                    //.setLensIntrinsics(1452.13,1452.13,653.281,181.125)
                    .build();
            VisionPortal visionPortal = new VisionPortal.Builder()
                    .addProcessor(tagProcessor)
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))

                    //.setCameraResolution(new Size(1280, 720))
                    .setCameraResolution(new Size(1920, 1080))
                    .build();
        }
        printRobotStatus();
        telemetry.addData("Status:", "Ready");
        //telemetry.addData("Driving Mode:", "Robot Centric");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        //robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

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
//                telemetry.addData("left_stick_x", gamepad1.left_stick_x);
//                telemetry.addData("left_stick_y", gamepad1.left_stick_y);
//                telemetry.addData("isReverse",isReverse);
//                telemetry.update();
                driveFieldCentric(strafePower, forwardPower, rotatePower, drivePower, offsetHeading);
            }

            //Co-Driver controller ---------------------
            if (gamepad2 != null) {
                if (gamepad2.a && robot.hasGrabberServo) {
                    if (isGrabberOpen) {
                        commands.servoSetPos(0.4);
                    } else {
                        commands.servoSetPos(0.6);
                    }
                    isGrabberOpen = !isGrabberOpen;
                    sleep(250);
                }

                if (gamepad1.y && robot.hasDroneServo) {
                    robot.droneServo.setPosition(1);
                    sleep(500);
                    robot.droneServo.setPosition(0);
                }

                if (robot.hasWristServo) {
                    if (gamepad2.dpad_down && grabberPosition == CenterStageEnums.Position.Up) {
                        robot.wristServo.setDirection(DcMotorSimple.Direction.FORWARD);
                        robot.wristServo.setPower(.3);
                        sleep(400);
                        robot.wristServo.setPower(0);
                        grabberPosition = CenterStageEnums.Position.Down;
                    }
                    if (gamepad2.dpad_up && grabberPosition == CenterStageEnums.Position.Down) {
                        robot.wristServo.setDirection(DcMotorSimple.Direction.REVERSE);
                        robot.wristServo.setPower(1);
                        if (isGrabberOpen) {
                            sleep(800);
                        } else {
                            sleep(1200);
                        }
                        robot.wristServo.setPower(0);
                        grabberPosition = CenterStageEnums.Position.Up;
                    }
                }


                armPower = -gamepad2.right_stick_y;
                if (robot.hasArmMotors) {
                    if (armPower > 0) {
                        setArmPower(armPower, CenterStageEnums.ArmDirection.Up);
                    } else if (armPower < 0) {
                        setArmPower(armPower, CenterStageEnums.ArmDirection.Down);
                    } else {
                        setArmPower(0, CenterStageEnums.ArmDirection.None);
                    }
                    // telemetry.addData("arm power:", armPower);
                    // telemetry.addData("arm position:", robot.armMotorRight.getCurrentPosition());
                }
                //telemetry.update();
            }
        }
    }

    private void setArmPower(double power, CenterStageEnums.ArmDirection armDirection) {
        if (!robot.hasArmMotors)
            return;
        // double ease = .4;
        double position = robot.armMotorRight.getCurrentPosition();
        telemetry.addData("arm power:", power);
        telemetry.addData("arm direction:", armDirection);
        telemetry.addData("arm position:", robot.armMotorRight.getCurrentPosition());

//        if (armDirection == ArmDirection.Up && position > 150) {
//            ease = .3;
//        }
//        if (armDirection == ArmDirection.Down && position < 150) {
//            ease = .3;
//        }
//        if (armDirection == ArmDirection.Down && position < 80) {
//            ease = 1;
//            power = .1;  // put the brakes on
//        }

        // power = power * ease;
        telemetry.addData("arm power eased:", power);
        telemetry.update();
        robot.armMotorRight.setPower(power);
        robot.armMotorLeft.setPower(power);
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
        return robot.imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        //return robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

}