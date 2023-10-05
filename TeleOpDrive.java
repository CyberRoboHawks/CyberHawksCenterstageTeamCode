package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
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
    HardwareMapping robot = new HardwareMapping();   // Use our hardware mapping
    Commands commands = new Commands();

    AprilTagProcessor tagProcessor = null;
    static final double STANDARD_DRIVE_SPEED = .3;
    static final double TURBO_DRIVE_SPEED = .6;
    static final double STRAFE_SPEED = .4;
    /*static final double LIFT_MAX_UP_POWER = .5;
    static final double LIFT_MAX_DOWN_POWER = .15;
    static final double LIFT_HOLD_POWER = .2;*/

    public enum ArmDirection {
        Down,
        None,
        Up
    }

    public enum FollowDirection {
        Rotate,
        Strafe,
        Straight
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
        boolean isreverse = false;
        boolean isServoOpen = true;
        boolean isGrabberOpen = true;
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

        waitForStart();

        telemetry.addData("Status:", "Ready");
        //telemetry.addData("Driving Mode:", "Robot Centric");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        //robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (!isStopRequested() && robot.hasCamera) {
                int targetId = 4;
                followTag(tagProcessor, FollowDirection.Straight, targetId);
                followTag(tagProcessor, FollowDirection.Strafe, targetId);
                followTag(tagProcessor, FollowDirection.Rotate, targetId);
            }

            double drivePower = STANDARD_DRIVE_SPEED;  //1 is 100%, .5 is 50%

            //Driver controller ---------------------
            if (gamepad1 != null) {
                if (gamepad1.left_bumper && gamepad1.right_bumper) {
                    telemetry.addData("hehe", true);
                    telemetry.update();
                }

                if (gamepad1.left_trigger > 0 && gamepad1.right_trigger > 0) {
                    telemetry.addData("triggers pressed", true);
                    telemetry.update();
                }

                if (gamepad1.a) {
                    if (isServoOpen) {
                        commands.servoSetPos(0.6);
                    } else {
                        commands.servoSetPos(0.8);
                    }
                    isServoOpen = !isServoOpen;
                    sleep(250);
                }
                if (gamepad1.y) {
                    commands.driveForward(.4, 12, 4, telemetry);
                    sleep(5000);
                }

                // Turbo driving
                if (gamepad1.left_bumper) {
                    drivePower = TURBO_DRIVE_SPEED;  // change drive speed to the turbo speed variable
                }

                if (gamepad1.a) {
                    commands.deliverPixel();
                }
                if (gamepad1.dpad_up) {
                    isreverse = robot.reverseMotorDirection();
                }

                // mecanum driving
                strafePower = gamepad1.left_stick_x;
                forwardPower = -gamepad1.left_stick_y;
                rotatePower = gamepad1.right_stick_x;
                if (isreverse) {
                    rotatePower *= -1;
                }
                offsetHeading = 0;

                driveFieldCentric(strafePower, forwardPower, rotatePower, drivePower, offsetHeading);
            }

            //Co-Driver controller ---------------------
            if (gamepad2 != null) {
                armPower = -gamepad2.right_stick_y;
                if (armPower > 0) {
                    setArmPower(armPower, ArmDirection.Up);
                } else if (armPower < 0) {
                    setArmPower(armPower, ArmDirection.Down);
                } else {
                    setArmPower(0, ArmDirection.None);
                }
                telemetry.addData("arm power:", armPower);
                telemetry.addData("arm position:", robot.armMotorRight.getCurrentPosition());
                telemetry.update();
            }
        }
    }

    private void setArmPower(double power, ArmDirection armDirection) {
        if (!robot.hasArmMotors)
            return;
        double ease = .6;
        double position = robot.armMotorRight.getCurrentPosition();
        if (armDirection == ArmDirection.Up && position > 150) {
            ease = .3;
        }
        if (armDirection == ArmDirection.Down && position < 150) {
            ease = .3;
        }
        if (armDirection == ArmDirection.Down && position < 80) {
            power = .1;  // put the brakes on
        }

        power = power * ease;
        robot.armMotorRight.setPower(power);
        robot.armMotorLeft.setPower(power);
    }

    private void followTag(AprilTagProcessor tagProcessor, FollowDirection direction, int targetId) {
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

            //tagProcessor.getDetections().indexOf(new AprilTagDetection(5));

            if (tag == null) {
                return;
            }
            telemetry.addData("ID", tag.id);
            telemetry.addData("angle x", tag.ftcPose.x);
            telemetry.addData("distance", tag.ftcPose.y);
            telemetry.addData("Z", tag.ftcPose.z);
//            telemetry.addData("Roll", tag.ftcPose.roll);
            telemetry.addData("angle Yaw", tag.ftcPose.yaw);
//            telemetry.addData("Pitch", tag.ftcPose.pitch);
//            telemetry.addData("Confidence:", tag.decisionMargin);
            telemetry.update();
            double speed = .3;
            double straightspeed = .3;
            double targetDistance = 12;
            double angle = tag.ftcPose.yaw;
            double x = tag.ftcPose.x;
            double distance = tag.ftcPose.y;
            double cameraOffset = 0;
            boolean currentside = true; // true is right size, false is left side
            if (direction == FollowDirection.Straight) {
                if (distance > targetDistance) {
                    commands.driveForward(straightspeed, (abs(distance) - targetDistance) / 2, 3, telemetry);
                }
            }
//            if (direction == FollowDirection.Rotate) {
//                if (tag.ftcPose.x > 5){
//                    commands.spinRight(speed, (getAngle() + abs(x))/2, 3);
//                } else if (tag.ftcPose.x < -5) {
//                    commands.spinLeft(speed, (getAngle() + abs(x))/2, 3);
//                }
//            }
            if (direction == FollowDirection.Rotate) {
                if (angle > 2) {
                    if (angle > 15) {
                        commands.strafeRight(speed, 4, 3);
                    }
                    commands.spinLeft(speed, getAngle() + angle / 3, 1);
                } else if (angle < -2) {
                    if (angle < -15) {
                        commands.strafeLeft(speed, 4, 3);
                    }
                    commands.spinRight(speed, getAngle() + angle / 3, 1);
                }
            }

            if (direction == FollowDirection.Strafe) {
                if (x > 3) {
                    commands.strafeRight(speed, (abs(x) - cameraOffset) / 3, 3);
                } else if (x < -3) {
                    commands.strafeLeft(speed, (abs(x) - cameraOffset) / 3, 3);
                }
            }
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
        return robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
}