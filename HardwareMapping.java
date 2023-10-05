package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class HardwareMapping {
    public boolean hasArmMotors = false;
    public boolean hasCamera = false;
    public boolean hasDriveMotors = false;
    public boolean hasGrabberServo = false;
    public boolean hasLinearActuatorMotor = false;
    public boolean hasPixelServo = false;

    public boolean isReverse = false;

    public DcMotor armMotorRight = null;
    public DcMotor armMotorLeft = null;
    public DcMotor leftFrontMotor = null;
    public DcMotor leftBackMotor = null;
    public DcMotor rightFrontMotor = null;
    public DcMotor rightBackMotor = null;
    public DcMotor linearActuatorMotor = null;
    public WebcamName webCam1 = null;
    public TouchSensor limitSwitchIn = null;
    public TouchSensor limitSwitchOut = null;
    public CRServo pixelServo = null;
    public Servo grabberServo = null;

    public BNO055IMU imu = null;

    HardwareMap hardwareMap = null;
    public ElapsedTime runtime = new ElapsedTime();

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hardware) {
        // Save reference to Hardware map
        hardwareMap = hardware;

        if (canGetDevice("armMotorRight"))
            armMotorRight = setupMotor("armMotorRight", DcMotor.Direction.FORWARD, 0, false,true);
        if (canGetDevice("armMotorLeft"))
            armMotorLeft = setupMotor("armMotorLeft", DcMotor.Direction.REVERSE, 0, false,true);
        hasArmMotors =  (canGetDevice("armMotorRight") && canGetDevice("armMotorLeft"));

        if (canGetDevice("Webcam 1")){
            webCam1 = setupWebcam("Webcam 1");
            hasCamera = true;
        }

        if (canGetDevice("leftFrontMotor") && canGetDevice("leftBackMotor")
                && canGetDevice("rightFrontMotor") && canGetDevice("rightBackMotor")){
            leftFrontMotor = setupMotor("leftFrontMotor", DcMotor.Direction.REVERSE, 0, true,true);
            leftBackMotor = setupMotor("leftBackMotor", DcMotor.Direction.REVERSE, 0, true,true);
            rightFrontMotor = setupMotor("rightFrontMotor", DcMotor.Direction.FORWARD, 0, true,true);
            rightBackMotor = setupMotor("rightBackMotor", DcMotor.Direction.FORWARD, 0, true,true);
            hasDriveMotors = true;
        }

        if (canGetDevice("pixelServo")){
            pixelServo = setupCRServo("pixelServo", 0.00);
            pixelServo.setDirection(DcMotorSimple.Direction.FORWARD);
            hasPixelServo = true;
        }

        if (canGetDevice("grabberServo")){
            grabberServo = setupServo("grabberServo",.5);
            hasGrabberServo = true;
        }

        if (canGetDevice("linearActuatorMotor")){
            linearActuatorMotor = setupMotor("linearActuatorMotor", DcMotor.Direction.FORWARD, 0, false,true);
            hasLinearActuatorMotor = true;
        }

        if (canGetDevice("limitSwitchIn"))
            limitSwitchIn = hardwareMap.get(TouchSensor.class, "limitSwitchIn");
        if (canGetDevice("limitSwitchOut"))
            limitSwitchOut = hardwareMap.get(TouchSensor.class, "limitSwitchOut");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    private boolean canGetDevice(String deviceName){
        try{
            hardwareMap.get(deviceName);
            return true;
        }
        catch (Exception ex){
            return false;
        }
    }
    public boolean reverseMotorDirection() {
        isReverse = !isReverse;
        if (isReverse) {
            leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
            leftBackMotor.setDirection(DcMotor.Direction.FORWARD);
            rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
            rightBackMotor.setDirection(DcMotor.Direction.REVERSE);
        } else {
            leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
            leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
            rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
            rightBackMotor.setDirection(DcMotor.Direction.FORWARD);
        }
        return isReverse;
    }

    /* Init Motor, set direction, initial power and encoder runmode (if applicable)
     * @return the configured DcMotor or null if the motor is not found
     */
    private DcMotor setupMotor(String name, DcMotorSimple.Direction direction, int initialPower, boolean useEncoder, boolean brakeMode) {
        try {

            DcMotor motor = hardwareMap.get(DcMotor.class, name);
            motor.setDirection(direction);
            motor.setPower(initialPower);

            if (useEncoder) {
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            if (brakeMode) {
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } else {
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
            return motor;
        } catch (Exception e) {
            return null;
        }
    }

    /* Init CRServo and set initial power
     * @return the configured CRServo or null if the servo is not found
     */
    private CRServo setupCRServo(String name, double initialPower) {
        try {
            CRServo servo = hardwareMap.get(CRServo.class, name);
            servo.setPower(initialPower);
            return servo;
        } catch (Exception e) {
            return null;
        }
    }

    /* Init Servo and set initial position
     * @return the configured Servo or null if the servo is not found
     */
    private Servo setupServo(String name, double initialPosition) {
        try {
            Servo servo = hardwareMap.get(Servo.class, name);
            servo.setPosition(initialPosition);
            return servo;
        } catch (Exception e) {
            return null;
        }
    }

    /* Init WebcamName
     * @return the configured WebcamName or null if the webcam is not found
     */
    private WebcamName setupWebcam(String name) {
        try {
            WebcamName webcamName = hardwareMap.get(WebcamName.class, name);
            return webcamName;
        } catch (Exception e) {
            return null;
        }
    }
}