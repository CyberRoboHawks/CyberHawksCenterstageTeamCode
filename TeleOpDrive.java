package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import android.util.Size;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


@TeleOp(name = "TeleOpDrive", group = "TeleOp") // add this code
//@Disabled
public class TeleOpDrive extends LinearOpMode {
    static final double STANDARD_DRIVE_SPEED = .4;
    static final double TURBO_DRIVE_SPEED = .8;

    private final ElapsedTime gametime = new ElapsedTime();
    HardwareMapping robot = new HardwareMapping();   // Use our hardware mapping
    Commands commands = new Commands(telemetry);

    AprilTagProcessor tagProcessor = null;

    private final ElapsedTime runtime = new ElapsedTime();
    /*static final double LIFT_MAX_UP_POWER = .5;
    static final double LIFT_MAX_DOWN_POWER = .15;
    static final double LIFT_HOLD_POWER = .2;*/

//    public void printRobotStatus() {
//        telemetry.addData("hasArmMotors: ", robot.hasArmMotors);
//        telemetry.addData("hasCamera: ", robot.hasCamera);
//        telemetry.addData("hasDriveMotors: ", robot.hasDriveMotors);
//        telemetry.addData("hasDroneServo: ", robot.hasDroneServo);
//        telemetry.addData("hasGrabberDistance: ", robot.hasGrabberDistance);
//        telemetry.addData("hasGrabberServo: ", robot.hasGrabberServo);
//        telemetry.addData("hasGripperSlideServo: ", robot.hasGripperSlideServo);
//        telemetry.addData("hasLinearActuatorMotor: ", robot.hasLinearActuatorMotor);
//        telemetry.addData("hasPixelServo: ", robot.hasPixelServo);
//        telemetry.addData("hasWristServo: ", robot.hasWristServo);
//    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the hardware variables.
        robot.init(hardwareMap);
        commands.init(hardwareMap);
        if (robot.isRoboHawks) {
            commands.reverseDriveMotorDirection();
        }
        double armPower;
        double strafePower;
        double forwardPower;
        double rotatePower;
        double offsetHeading = 0;
        boolean isReverse = false;
        boolean isGrabberOpen = false;
        //boolean isWristUp = true;
        //CenterStageEnums.Position grabberPosition = CenterStageEnums.Position.Up;

//        if (robot.hasCamera) {
//            tagProcessor = new AprilTagProcessor.Builder()
//                    .setDrawCubeProjection(true)
//                    .setDrawTagID(true)
//                    .build();
//            VisionPortal visionPortal = new VisionPortal.Builder()
//                    .addProcessor(tagProcessor)
//                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                    .setCameraResolution(new Size(1920, 1080))
//                    .build();
//        }
        telemetry.addData("Status:", "Ready");
        commands.printRobotStatus(telemetry);
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        gametime.reset();

        //robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (robot.hasBlinkin){
                if (robot.armPosition == CenterStageEnums.Position.Up
                        && robot.hasGrabberDistance
                        && robot.grabberDistance.getDistance(DistanceUnit.CM) <= 6) {
                    robot.blinkinLedDriver.setPattern(BlinkinPattern.GREEN);
                }
                else if (gametime.seconds() >= 90){
                    robot.blinkinLedDriver.setPattern(BlinkinPattern.HEARTBEAT_RED);
                }
                else{
                    robot.blinkinLedDriver.setPattern(BlinkinPattern.RAINBOW_OCEAN_PALETTE);
                }
            }

            double drivePower = STANDARD_DRIVE_SPEED;  //1 is 100%, .5 is 50%

            //Driver controller ---------------------
            if (gamepad1 != null) {
                if (gamepad1.dpad_up && gamepad1.y) {
                    isReverse = commands.reverseDriveMotorDirection();
                    sleep(250);
                }

                if (gamepad1.left_trigger > 0 && gamepad1.right_trigger > 0) {
                    telemetry.addData("triggers pressed", true);
                    telemetry.update();
                }

                // Turbo driving
                if (gamepad1.left_bumper || gamepad1.right_bumper) {
                    drivePower = TURBO_DRIVE_SPEED;  // change drive speed to the turbo speed variable
                }

//                // test - will only be auton action
//                if (gamepad1.a && robot.hasPixelServo) {
//                    commands.deliverGroundPixel();
//                }

                // test
//                if (gamepad1.dpad_up && robot.hasArmMotors && robot.isRoboHawks) {
//                    commands.setArmPositionRH(CenterStageEnums.ArmDirection.Up, telemetry);
//                    sleep(250);
//                }
//                if (gamepad1.dpad_down && robot.hasArmMotors && robot.isRoboHawks) {
//                    commands.setArmPositionRH(CenterStageEnums.ArmDirection.Down, telemetry);
//                    sleep(250);
//                    commands.setArmPower(0);
//                }
//                if (robot.armPositionTarget != robot.armMotorRight.getCurrentPosition()){
//                    telemetry.addData("current arm pos", robot.armMotorRight.getCurrentPosition());
//                    telemetry.addData("current arm target",robot.armPositionTarget);
//                    telemetry.update();
//                    //commands.setArmPosition(robot.armPositionTarget);
//                }
//
//                // test - will only be auton action
//                if (gamepad1.x && robot.hasWristServo) {
//                    commands.setWristPositionBackdrop();
//                }
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
                if (commands.hasGripperSlideServo) {
                    if (gamepad2.dpad_left) {
                        robot.gripperSlideServo.setDirection(DcMotorSimple.Direction.FORWARD);
                        robot.gripperSlideServo.setPower(.5);
                        sleep(100);
                    } else if (gamepad2.dpad_right) {
                        robot.gripperSlideServo.setDirection(DcMotorSimple.Direction.REVERSE);
                        robot.gripperSlideServo.setPower(.5);
                        sleep(100);
                    }

                    robot.gripperSlideServo.setPower(0);
                }

                if (gamepad2.a && robot.hasGrabberServo) {
                    if (isGrabberOpen) {
                        commands.setGrabberPosition(robot.GRABBER_CLOSED);
                    } else {
                        commands.setGrabberPosition(robot.GRABBER_OPEN);
                    }
                    isGrabberOpen = !isGrabberOpen;
                    sleep(250);
                }

                // prevent early launch with gameTimer check for endgame
                // switch back to gamepad2 later
                if (gamepad2.y && robot.hasDroneServo){
                        //&& ((gamepad2.y && gametime.seconds() > 90){
                    commands.launchDrone();
                }

                if (gamepad2.x && robot.hasWristServo) {
                    commands.reverseWristPosition();
                }

                armPower = -gamepad2.left_stick_y;
                if (armPower !=0){
                    if (armPower > 0){
                        if (robot.isRoboHawks){
                            commands.setArmPositionRH(CenterStageEnums.ArmDirection.Up);
                        }else
                        {
                            commands.setArmPosition(CenterStageEnums.ArmDirection.Up, 3);
                        }
                        robot.armPosition= CenterStageEnums.Position.Up;
                    }
                    else{
                        if (robot.isRoboHawks){
                            commands.setArmPositionRH(CenterStageEnums.ArmDirection.Down);
                        }else{
                            commands.setWristPosition(CenterStageEnums.Position.Up);
                            commands.setArmPosition(CenterStageEnums.ArmDirection.Down, 3);
                            commands.setGrabberPosition(robot.GRABBER_OPEN);
                        }
                        robot.armPosition = CenterStageEnums.Position.Down;
                    }
                }

                if ( robot.hasLinearActuatorMotor ){
                    if (gamepad2.right_stick_y !=0){
                        commands.moveLinearActuator(-gamepad2.right_stick_y,false);
                    }
                    // Allow override of linear actuator limits
                    if (gamepad2.right_stick_y !=0 && gamepad2.left_bumper){
                        commands.moveLinearActuator(-gamepad2.right_stick_y,true);
                    }
                }
            }
        }
        telemetry.update();
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