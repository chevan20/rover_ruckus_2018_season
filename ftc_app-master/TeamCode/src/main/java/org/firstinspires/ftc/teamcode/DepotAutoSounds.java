package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

@Autonomous(name = "ColeeUploadTestThing", group = "test")
public class DepotAutoSounds extends LinearOpMode {

    int goldMineralX = -100;
    int degreeOfAccuracy = 2;

    double dPos = 0;

    private DcMotor FL = null;
    private DcMotor FR = null;
    private DcMotor BR = null;
    private DcMotor BL = null;
    private DcMotor HookMotor = null;
    private boolean Left = false, Right = false, Center = false;
    ModernRoboticsI2cGyro gyro;
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AT0ySZn/////AAABGVDoF4gdNkZugx61WeftYqVhwz6Leeu2a1cWYQR+08xsATI6GQf3vvrvynP8JpemukCajxoFg32bkspzJx8g6uNBgHlQsFPxmFMJ8b4V/fDFTRSpy+vMOzIMoV2CHuitvtyrn/a6AsPUWczm5rsTqcCzAUEL6YD0xrzXkvaNJBzm3Jq5BkUW2ualta+LldpZ0ho/rdkDuyp6xOjsSvAbsjIDkQt807jlfYLBJAsaJNqRnQUU4mZMzl5aJsr+VnUbTfeev943zeK34ENEQzW1jCeCnLTsuWGmOd7QP+gF1YhxUr0A0s5Tr6+v8QntHPwYHp7QRpRErFqkst/OxbA4omIuLTsC41FFwYOr5YxDHhyl";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private Servo TeamMarkerServo;

    private void FaB(double pow) {
        FL.setPower(-pow);
        FR.setPower(-pow);
        BL.setPower(-pow);
        BR.setPower(-pow);
    }

    private void Turn(double pow) {
        FL.setPower(pow);
        FR.setPower(-pow);
        BL.setPower(pow);
        BR.setPower(-pow);
    }

    private void StopWheels() {
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }

    public void Hook(double dist) {

        double gearRatio = 18 / 22;// ratio between HookMotor and the leadscrew
        double leadScrewPitch = 8 / 25.4; //Leadscrew pitch converted from millimeters to inches
        double stepsPerRotation = 1680; // Steps per rotation for whatever gear ratio the hook motor is

        double stepsPerInchLinear = (gearRatio * stepsPerRotation) / leadScrewPitch; // This gives the number of steps to move one inch on the linear axis

        double stepsToMoveForDist = stepsPerInchLinear * dist; // Number of steps to move for a given distance
        if (dist > 0) {
            while (stepsToMoveForDist > HookMotor.getCurrentPosition() && !isStopRequested()) { // Move up while the encoder value is below the amount it needs to get to; as long as stop is not requested
                HookMotor.setPower(1);
                telemetry.addData("FH", HookMotor.getCurrentPosition());
                telemetry.update();
            }
        } else if (dist < 0) {
            while (stepsToMoveForDist < HookMotor.getCurrentPosition() && !isStopRequested()) { // Move down while the encoder value is above where it needs to get to while a stop is not requested
                HookMotor.setPower(-1);
            }
        }
        HookMotor.setPower(0);
    }

    public void resetEncoder() {

        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    //POSITION 0.025
    private void servoUp(double POSITION) {
        dPos += POSITION;
        TeamMarkerServo.setPosition(dPos);

        if (dPos > 1) {
            dPos = 1;
        } else if (dPos < 0) {
            dPos = 0;
        }

    }

    private void servoDown(double POSITION) {
        dPos -= POSITION;
        TeamMarkerServo.setPosition(dPos);

        if (dPos > 1) {
            dPos = 1;
        } else if (dPos < 0) {
            dPos = 0;
        }
    }

    public void moveDistance(double length) {
        resetEncoder();
        double distPerRot = Math.PI * 3.8125;
        double stepsPerRot = 560;
        double totDistInSteps = ((length / distPerRot) * stepsPerRot) * -1;
        if (length > 0) {
            FaB(.5);

            while (totDistInSteps <= FL.getCurrentPosition() && !isStopRequested()) {

                telemetry.addData("Position", FL.getCurrentPosition());
                telemetry.addData("tot", totDistInSteps);
                telemetry.update();


            }
            StopWheels();

        } else if (length < 0) {
            FaB(-.5);

            while (totDistInSteps >= FL.getCurrentPosition() && !isStopRequested()) {

                telemetry.addData("Position", FL.getCurrentPosition());
                telemetry.addData("tot", totDistInSteps);
                telemetry.update();


            }
            StopWheels();

        }
    }

    public void turnDegree(int degree) {
        gyro.resetZAxisIntegrator();
        resetEncoder();
        if (degree <= 180) {
            while (!(gyro.getHeading() > degree - degreeOfAccuracy && gyro.getHeading() < degree + degreeOfAccuracy) && !isStopRequested()) {
                Turn(-.1);
            }
            StopWheels();
        } else if (degree > 180) {
            while (!(gyro.getHeading() > degree - degreeOfAccuracy && gyro.getHeading() < degree + degreeOfAccuracy) && !isStopRequested()) {
                Turn(.1);
            }
            StopWheels();
        }
    }

    public void dropTeamMarker() {
        while (dPos < 1 && !isStopRequested()) {
            servoDown(1);
        }
        while (dPos > 0 && !isStopRequested()) {
            servoUp(1);
        }
    }

    public void runOpMode() {

        FL = hardwareMap.get(DcMotor.class, "FL");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BR = hardwareMap.get(DcMotor.class, "BR");

        HookMotor = hardwareMap.get(DcMotor.class, "Hook");

        gyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");

        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);

        TeamMarkerServo = hardwareMap.get(Servo.class, "d");

        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        HookMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        HookMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        HookMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        gyro.calibrate();
        while (gyro.isCalibrating()) {

        }
        gyro.resetZAxisIntegrator();
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();

        waitForStart();

        if (!isStopRequested()) {

            while (HookMotor.getCurrentPosition() < 11800 && !isStopRequested()) {
                HookMotor.setPower(1);
                telemetry.addData("Hook", HookMotor.getCurrentPosition());
                telemetry.update();
                turnDegree(5);
            }
            HookMotor.setPower(0);
            moveDistance(10);
        }


        while (!isStopRequested()) {


        }


    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);

    }

}
