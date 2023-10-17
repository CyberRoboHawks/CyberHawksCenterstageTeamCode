package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;
import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.function.Supplier;

public class Commands extends HardwareMapping {
    //https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
    static final double COUNTS_PER_MOTOR_REV = 537.7;//384.5;    // eg: TETRIX Motor Encoder (1440 - 60:1; 960 - 40:1, 480 - 20:1)
    static final double WHEEL_DIAMETER_INCHES = 3.75;   // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * Math.PI);

    private final ElapsedTime runtime = new ElapsedTime();

    public void servoSetPos(double position) {
        grabberServo.setPosition(position);
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

    // Drive forward
    public void driveForward(double power, double distanceInInches, double timeout, Telemetry telemetry) {
        encoderDriveStraight(power, distanceInInches, timeout, telemetry);
    }

    // Drive backward
    public void driveBackwards(double power, double distanceInInches, double timeout, Telemetry telemetry) {
        encoderDriveStraight(power, -distanceInInches, timeout, telemetry);
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

    // Strafe left
    public void strafeLeft(double power, double distanceInInches, double timeout) {
        encoderDriveStrafe(power, distanceInInches, strafeDirection.Left, timeout);
    }

    // Strafe right
    public void strafeRight(double power, double distanceInInches, double timeout) {
        encoderDriveStrafe(power, distanceInInches, strafeDirection.Right, timeout);
    }

    private void encoderDriveStrafe(double power, double distanceInches, strafeDirection direction, double timeoutS) {
        int leftBackTarget = 0;
        int leftFrontTarget = 0;
        int rightBackTarget = 0;
        int rightFrontTarget = 0;

        // strafe offset to add 140% for difference in requested and actual movement
        //distanceInches = distanceInches * 1.40;
        int newMotorPosition = (int) (distanceInches * COUNTS_PER_INCH);

        // Determine new target position, and pass to motor controller
        if (direction == strafeDirection.Left) {
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

    private double getRemainingAngle(double targetAngle) {
        // calculate angle in -179 to +180 range  (
        // Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        Orientation angles = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return targetAngle - angles.firstAngle;
    }

    private double getAngle() {
        //return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        return imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
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


    public boolean reverseMotorDirection() {
        leftFrontMotor.setDirection(leftFrontMotor.getDirection().inverted());
        leftBackMotor.setDirection(leftBackMotor.getDirection().inverted());
        rightFrontMotor.setDirection(rightFrontMotor.getDirection().inverted());
        rightBackMotor.setDirection(rightBackMotor.getDirection().inverted());

        isReverse = !isReverse;
        return isReverse;
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

    public void deliverPixel() throws InterruptedException {
        pixelServo.setPosition(.4);
        sleep(100);
        pixelServo.setPosition(.6);
        sleep(100);
        pixelServo.setPosition(.8);
        sleep(300);
        pixelServo.setPosition(.2);
//        pixelServo.setPower(.2);
//        sleep(1265);
//        pixelServo.setPower(0);
    }

    private void stopDrivingMotors() {
        leftBackMotor.setPower(0);
        leftFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
    }

    public enum strafeDirection {
        Right,
        Left
    }
}