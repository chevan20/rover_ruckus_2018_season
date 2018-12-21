package org.firstinspires.ftc.teamcode;

import android.media.AudioManager;
import android.media.SoundPool;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name = "Crater Auto", group = "test")
public class CraterAuto extends LinearOpMode {

    int goldMineralX = -100;
    int degreeOfAccuracy = 2;

    double dPos = 0;

    public SoundPool mySound;
    public int yeetID;
    public int trolol;
    public int gotem;

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

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        mySound = new SoundPool(1, AudioManager.STREAM_MUSIC, 0); // PSM
        yeetID = mySound.load(hardwareMap.appContext, R.raw.yeit, 1); // PSM
        trolol = mySound.load(hardwareMap.appContext, R.raw.trolol, 1);
        gotem = mySound.load(hardwareMap.appContext, R.raw.hagotem, 1);

        if (tfod != null) {
            tfod.activate();
        }

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

        gyro.resetZAxisIntegrator();
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();

        waitForStart();

        if (!isStopRequested()) {
            mySound.play(trolol, 1, 1, 1, 3, 1);
            while (HookMotor.getCurrentPosition() < 16000 && !isStopRequested()) {
                HookMotor.setPower(1);
                telemetry.addData("Hook", HookMotor.getCurrentPosition());
                telemetry.update();
            }
            HookMotor.setPower(0);
            moveDistance(2);
            gyro.calibrate();
            while (gyro.isCalibrating()) {
            }

        }


        while (!isStopRequested()) {

            telemetry.addData("TFOD", "RUNNING");
            telemetry.update();

            if (Right && !isStopRequested()) {
                //-----------------------------------------------------------------------------RIGHT
                telemetry.addData("Right", goldMineralX);
                telemetry.update();
                turnDegree(320);
                moveDistance(40);
                moveDistance(-20);
                stop();
            } else if (Center && !isStopRequested()) {//DOne
                //----------------------------------------------------------------------------CENTER
                telemetry.addData("Center", goldMineralX);
                telemetry.update();
                moveDistance(48);
                moveDistance(-20);
                stop();
            } else if (Left && !isStopRequested()) {
                //------------------------------------------------------------------------------LEFT
                telemetry.addData("Left", goldMineralX);
                telemetry.update();
                turnDegree(30);
                moveDistance(40);
                moveDistance(-20);
                stop();
            } else {

                //------------------------------------------------------------------------------TFOD
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() != 0) {
                            for (Recognition recognition : updatedRecognitions) {

                                for (int i = 0; i < 2 && !isStopRequested(); i++) {

                                    sleep(1000);//--------------------------------
                                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                        goldMineralX = (int) recognition.getTop();
                                        telemetry.addData("Mineral Location", goldMineralX);
                                        telemetry.update();
                                        if (goldMineralX > -100 && goldMineralX < 600) {
                                            Right = true;
                                        } else if (goldMineralX > 600 && goldMineralX < 1200) {
                                            Center = true;
                                        }
                                    }
                                }
                                if (Right == false && Center == false) {
                                    Left = true;
                                }
                            }


                        }

                    }
                    telemetry.update();
                }

            }

        }
        if (tfod != null) {
            tfod.shutdown();
        }
        if (isStopRequested()) {
            mySound.autoPause();
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
