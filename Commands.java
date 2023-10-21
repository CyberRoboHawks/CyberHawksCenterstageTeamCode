package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;
import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.function.Supplier;

public class Commands extends HardwareMapping {
    //https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
    static final double COUNTS_PER_MOTOR_REV = 537.7;//384.5;    // eg: TETRIX Motor Encoder (1440 - 60:1; 960 - 40:1, 480 - 20:1)
    static final double WHEEL_DIAMETER_INCHES = 3.75;   // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * Math.PI);

    static final double STRAFE_SPEED = .4;

    private final ElapsedTime runtime = new ElapsedTime();

    public void deliverGroundPixel() throws InterruptedException {
        pixelServo.setPosition(.4);
        sleep(100);
        pixelServo.setPosition(.6);
        sleep(100);
        pixelServo.setPosition(.8);
        sleep(300);
        pixelServo.setPosition(.2);
    }

    public void driveBackwards(double power, double distanceInInches, double timeout, Telemetry telemetry) {
        encoderDriveStraight(power, -distanceInInches, timeout, telemetry);
    }

    public void driveForward(double power, double distanceInInches, double timeout, Telemetry telemetry) {
        encoderDriveStraight(power, distanceInInches, timeout, telemetry);
    }

    public void driveMecanum(double strafeSpeed, double forwardSpeed, double rotate, double drivePower, double heading) {
        if (!hasDriveMotors)
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
        leftFrontMotor.setPower(leftFrontPower);
        leftBackMotor.setPower(leftBackPower);
        rightFrontMotor.setPower(rightFrontPower);
        rightBackMotor.setPower(rightBackPower);
    }

    public void driveStraightWhile(double power, Supplier<Boolean> keepGoing, double timeout) throws InterruptedException {
        if (!keepGoing.get()) {
            return;
        }
        runtime.reset();
        setMotorRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftBackMotor.setPower(abs(power));
        leftFrontMotor.setPower(abs(power));
        rightBackMotor.setPower(abs(power));
        rightFrontMotor.setPower(abs(power));

        while (keepGoing.get() && runtime.seconds() < timeout) {
            sleep(100);
        }

        // Stop all motion;
        stopDrivingMotors();
        setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void encoderRunToPosition(double power, int leftFrontTarget, int leftBackTarget, int rightFrontTarget, int rightBackTarget, double timeoutS) {
        leftFrontMotor.setTargetPosition(leftFrontTarget);
        leftBackMotor.setTargetPosition(leftBackTarget);
        rightFrontMotor.setTargetPosition(rightFrontTarget);
        rightBackMotor.setTargetPosition(rightBackTarget);

        // Turn On RUN_TO_POSITION
        setMotorRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        leftBackMotor.setPower(abs(power));
        leftFrontMotor.setPower(abs(power));
        rightBackMotor.setPower(abs(power));
        rightFrontMotor.setPower(abs(power));

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (
                (runtime.seconds() < timeoutS) &&
                        (leftBackMotor.isBusy()
                                && leftFrontMotor.isBusy()
                                && rightBackMotor.isBusy()
                                && rightFrontMotor.isBusy())) {
        }

        // Stop all motion;
        stopDrivingMotors();

        // Turn off RUN_TO_POSITION
        setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void encoderDriveStraight(double power, double distanceInches, double timeoutS, Telemetry telemetry) {

        int newMotorPosition = (int) (distanceInches * COUNTS_PER_INCH);
        int leftBackTarget = leftBackMotor.getCurrentPosition() + newMotorPosition;
        int leftFrontTarget = leftFrontMotor.getCurrentPosition() + newMotorPosition;
        int rightBackTarget = rightBackMotor.getCurrentPosition() + newMotorPosition;
        int rightFrontTarget = rightFrontMotor.getCurrentPosition() + newMotorPosition;
//        telemetry.addData("Distance",distanceInches);
//        telemetry.addData("MotorPos",newMotorPosition);
//        telemetry.addData("leftBack",leftBackTarget);
//        telemetry.addData("leftFront",leftFrontTarget);
//        telemetry.addData("rightBack",rightBackTarget);
//        telemetry.addData("rightFront",rightFrontTarget);
//        telemetry.update();
        // Determine new target positions, and pass to motor controller

        encoderRunToPosition(power, leftFrontTarget, leftBackTarget, rightFrontTarget, rightBackTarget, timeoutS);
    }


    private void encoderDriveStrafe(double power, double distanceInches, CenterStageEnums.StrafeDirection direction, double timeoutS) {
        int leftBackTarget = 0;
        int leftFrontTarget = 0;
        int rightBackTarget = 0;
        int rightFrontTarget = 0;

        // strafe offset to add 140% for difference in requested and actual movement
        //distanceInches = distanceInches * 1.40;
        int newMotorPosition = (int) (distanceInches * COUNTS_PER_INCH);

        // Determine new target position, and pass to motor controller
        if (direction == CenterStageEnums.StrafeDirection.Left) {
            //Left strafe
            leftBackTarget = leftBackMotor.getCurrentPosition() + newMotorPosition;
            leftFrontTarget = leftFrontMotor.getCurrentPosition() - newMotorPosition;
            rightBackTarget = rightBackMotor.getCurrentPosition() - newMotorPosition;
            rightFrontTarget = rightFrontMotor.getCurrentPosition() + newMotorPosition;

        } else {
            //Right strafe
            leftBackTarget = leftBackMotor.getCurrentPosition() - newMotorPosition;
            leftFrontTarget = leftFrontMotor.getCurrentPosition() + newMotorPosition;
            rightBackTarget = rightBackMotor.getCurrentPosition() + newMotorPosition;
            rightFrontTarget = rightFrontMotor.getCurrentPosition() - newMotorPosition;
        }

        encoderRunToPosition(power, leftFrontTarget, leftBackTarget, rightFrontTarget, rightBackTarget, timeoutS);
    }

    public void followTag(AprilTagProcessor tagProcessor, CenterStageEnums.FollowDirection direction, int targetId, Telemetry telemetry) throws InterruptedException {
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
                    driveForward(straightspeed, (abs(distance) - targetDistance) * straightGain, 3, telemetry);
                }
                if (heading > 20) {
                    driveBackwards(straightspeed, 3, 3, telemetry);
                    strafeLeft(speed, 2, 2);
                }
                if (heading < -20) {
                    driveBackwards(straightspeed, 3, 3, telemetry);
                    strafeRight(speed, 2, 2);
                }
            }

            if (direction == CenterStageEnums.FollowDirection.Rotate) {
                double targetAngle = getAngle() + heading * angleGain;
                if (heading > 4) {
                    spinLeft(speed, targetAngle, 1);
                } else if (heading < -4) {
                    spinRight(speed, targetAngle, 1);
                }
            }

            if (direction == CenterStageEnums.FollowDirection.Strafe) {
                double strafeTarget = (abs(yaw) * strafeGain - cameraOffset);
                if (yaw > 4) {
                    strafeRight(speed, strafeTarget, 3);
                } else if (yaw < -4) {
                    strafeLeft(speed, strafeTarget, 3);
                }
            }
            sleep(50);
        }
    }

    private double getAngle() {
        //return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        return imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    private double getRemainingAngle(double targetAngle) {
        // calculate angle in -179 to +180 range  (
        // Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        Orientation angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return targetAngle - angles.firstAngle;
    }

    public CenterStageEnums.TapeColor getTapeColor(float hue, float saturation, float value) {
        if (hue > 205 && hue < 230) {
            return CenterStageEnums.TapeColor.Blue;
        }
        if (hue > 21 && hue < 38) {
            return CenterStageEnums.TapeColor.Red;
        }
        return CenterStageEnums.TapeColor.Nothing;
    }

    private void gyroTurn(double leftMotorPower, double rightMotorPower, double heading, double timeout) {
        runtime.reset();

        while (abs(getRemainingAngle(heading)) > 1 && (runtime.seconds() < timeout)) {
            leftFrontMotor.setPower(leftMotorPower);
            leftBackMotor.setPower(leftMotorPower);
            rightFrontMotor.setPower(rightMotorPower);
            rightBackMotor.setPower(rightMotorPower);
        }
        stopDrivingMotors();
    }

    public void launchDrone() throws InterruptedException {
        droneServo.setPosition(1);
        sleep(500);
        droneServo.setPosition(0);
    }

    public void quickSpin(double power, double heading, double timeout) {
        double currentAngle = getAngle();
        if (currentAngle < 0) {
            if (heading < currentAngle || (currentAngle < -80 && heading > 90)) {
                if (heading == 180) {
                    heading = -179;
                }
                spinRight(power, heading, timeout);
            } else {
                spinLeft(power, heading, timeout);
            }
        } else {
            if (heading < currentAngle || (currentAngle > 80 && heading < -90)) {
                spinRight(power, heading, timeout);
            } else {
                spinLeft(power, heading, timeout);
            }
        }
    }

    public boolean reverseDriveMotorDirection() {
        leftFrontMotor.setDirection(leftFrontMotor.getDirection().inverted());
        leftBackMotor.setDirection(leftBackMotor.getDirection().inverted());
        rightFrontMotor.setDirection(rightFrontMotor.getDirection().inverted());
        rightBackMotor.setDirection(rightBackMotor.getDirection().inverted());

        isReverse = !isReverse;
        return isReverse;
    }

    public void reverseWristPosition() throws InterruptedException {
        double distanceCm = grabberDistance.getDistance(DistanceUnit.CM);
        if (distanceCm > 10) { //Move down
            wristServo.setPosition(.13);
        } else { //move up
            wristServo.setPosition(0.7);
        }
        sleep(250);
    }

    public void setWristPositionBackdrop() throws InterruptedException {
        wristServo.setPosition(0.59);
        sleep(250);
    }

    public void setGrabberPosition(double position) {
        grabberServo.setPosition(position);
    }

    public void setMotorRunMode(DcMotor.RunMode mode) {
        leftBackMotor.setMode(mode);
        leftFrontMotor.setMode(mode);
        rightBackMotor.setMode(mode);
        rightFrontMotor.setMode(mode);
    }

    // Rotate left (counter-clockwise)
    public void spinLeft(double power, double heading, double timeout) {
        gyroTurn(-power, power, heading, timeout);
    }

    // Rotate right (clockwise)
    public void spinRight(double power, double heading, double timeout) {
        gyroTurn(power, -power, heading, timeout);
    }

    public void strafeLeft(double power, double distanceInInches, double timeout) {
        encoderDriveStrafe(power, distanceInInches, CenterStageEnums.StrafeDirection.Left, timeout);
    }

    public void strafeRight(double power, double distanceInInches, double timeout) {
        encoderDriveStrafe(power, distanceInInches, CenterStageEnums.StrafeDirection.Right, timeout);
    }


    public void setArmPosition(CenterStageEnums.ArmDirection armDirection, double timeout, Telemetry telemetry) {
        if (!hasArmMotors)
            return;

        double position = 0;
        double power = 0;
        boolean isArmMoving = true;

        runtime.reset();

        while (isArmMoving && (runtime.seconds() < timeout)) {
            position = armMotorRight.getCurrentPosition();// + robot.armMotorLeft.getCurrentPosition()) / 2.0;
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
            armMotorRight.setPower(power);
            armMotorLeft.setPower(power);
            if (power == 0) {
                isArmMoving = false;
            }
        }
        position = armMotorRight.getCurrentPosition();// + robot.armMotorLeft.getCurrentPosition()) / 2.0;

        telemetry.addData("position:", position);
        telemetry.addData("arm power:", power);
        telemetry.update();
    }

    private void stopDrivingMotors() {
        leftBackMotor.setPower(0);
        leftFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
    }

}