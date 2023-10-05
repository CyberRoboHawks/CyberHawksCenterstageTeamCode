package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "TeleOpLinearActuator", group = "TeleOp") // add this code
//@Disabled
public class TeleOpLinearActuator extends LinearOpMode {
    HardwareMapping robot = new HardwareMapping();   // Use our hardware mapping

    public void move(double linearActuatorPower){
        if (linearActuatorPower > 0 && !robot.limitSwitchOut.isPressed()) {
            robot.linearActuatorMotor.setPower(linearActuatorPower);
        }
        else if(linearActuatorPower<0 && !robot.limitSwitchIn.isPressed()) {
            robot.linearActuatorMotor.setPower(linearActuatorPower);
        }
        else {
            robot.linearActuatorMotor.setPower(0);
        }
    }


    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the hardware variables.
        robot.init(hardwareMap);

        double linearActuatorPower = 0;
        telemetry.addData("Status:", "Ready");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

       // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (!isStopRequested()) {

                // send the info back to driver station using telemetry function.
                if (robot.limitSwitchIn.isPressed()) {
                    telemetry.addData("Limit In: ", "Is Pressed");
                } else {
                    telemetry.addData("Limit In: ", "Is Not Pressed");
                }
                if (robot.limitSwitchOut.isPressed()) {
                    telemetry.addData("Limit Out: ", "Is Pressed");
                } else {
                    telemetry.addData("Limit Out: ", "Is Not Pressed");
                }
                telemetry.update();

                //Co-Driver controller ---------------------
                if (gamepad2 != null) {

                    linearActuatorPower  = -gamepad2.left_stick_y;  //pushing forward gives a negative value

                    // TODO Move to in position

                    move(linearActuatorPower);
//                    if (linearActuatorPower > 0 && !robot.limitSwitchOut.isPressed()) {
//                        robot.linearActuatorMotor.setPower(linearActuatorPower);
//                    }
//                    else if(linearActuatorPower<0 && !robot.limitSwitchIn.isPressed()) {
//                        robot.linearActuatorMotor.setPower(linearActuatorPower);
//                    }
//                    else {
//                        robot.linearActuatorMotor.setPower(0);
//                    }
                }
            }
        }
    }
}