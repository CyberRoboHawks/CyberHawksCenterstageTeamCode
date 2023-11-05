/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection, using
 * the easiest way.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@TeleOp(name = "Concept: TensorFlow Object Detection Easy", group = "Concept")

//@Disabled
public class ConceptTensorFlowObjectDetectionEasy extends LinearOpMode {
    Commands commands = new Commands(telemetry);
    HardwareMapping robot = new HardwareMapping();   // Use our hardware mapping
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;


    ExposureControl myExposureControl;  // declare exposure control object
    long minExp;
    long maxExp;
    long curExp;            // exposure is duration, in time units specified

    GainControl myGainControl;      // declare gain control object
    int minGain;
    int maxGain;
    int curGain;
    boolean wasSetGainSuccessful;   // returned from setGain()
CenterStageEnums.TapeLocation tapeLocation = CenterStageEnums.TapeLocation.None;

    @Override
    public void runOpMode() {

        initTfod();

        // *** ADD WEBCAM CONTROLS -- SECTION START ***

        // Assign the exposure and gain control objects, to use their methods.
        myExposureControl = visionPortal.getCameraControl(ExposureControl.class);
        myGainControl = visionPortal.getCameraControl(GainControl.class);

        // get webcam exposure limits
        minExp = myExposureControl.getMinExposure(TimeUnit.MILLISECONDS);
        maxExp = myExposureControl.getMaxExposure(TimeUnit.MILLISECONDS);

        // get webcam gain limits
        minGain = myGainControl.getMinGain();
        maxGain = myGainControl.getMaxGain();

        // Change mode to Manual, in order to control directly.
        // A non-default setting may persist in the camera, until changed again.
        myExposureControl.setMode(ExposureControl.Mode.Manual);

        // Retrieve from webcam its current exposure and gain values
        curExp = myExposureControl.getExposure(TimeUnit.MILLISECONDS);
        curGain = myGainControl.getGain();

        // display exposure mode and starting values to user
        telemetry.addLine("\nTouch Start arrow to control webcam Exposure and Gain");
        telemetry.addData("\nCurrent exposure mode", myExposureControl.getMode());
        telemetry.addData("Current exposure value", curExp);
        telemetry.addData("Current gain value", curGain);
        telemetry.update();
        // *** ADD WEBCAM CONTROLS -- SECTION END ***

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                //telemetryTfod();
                tapeLocation = getTfodRecognitions(tfod);

                // Push telemetry to the Driver Station.
                telemetry.update();

                // Save CPU resources; can resume streaming when needed.
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                }

                // Share the CPU.
                sleep(20);
            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end runOpMode()

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor the easy way.
        tfod = TfodProcessor.easyCreateWithDefaults();
        tfod.setMinResultConfidence(.5f);

        // Create the vision portal the easy way.

        visionPortal = new VisionPortal.Builder()
                .addProcessor(tfod)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(1920, 1080))
                .build();

    }
    // end method initTfod()

    private CenterStageEnums.TapeLocation getTfodRecognitions(TfodProcessor tfod) {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        //if (currentRecognitions == null) currentRecognitions = tfod.getFreshRecognitions();
        if (currentRecognitions == null) return CenterStageEnums.TapeLocation.None;

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            telemetry.update();

            if (x < 500) return CenterStageEnums.TapeLocation.Left;
            if (x > 500 && x < 1500) return CenterStageEnums.TapeLocation.Center;
            if (x > 1500) return CenterStageEnums.TapeLocation.Right;
        }   // end for() loop

        return  CenterStageEnums.TapeLocation.None;
    }

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;

            telemetry.addData("", " ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }   // end method telemetryTfod()

}   // end class
