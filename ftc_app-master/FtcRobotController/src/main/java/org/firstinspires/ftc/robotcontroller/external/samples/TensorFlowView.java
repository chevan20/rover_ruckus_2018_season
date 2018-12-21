/* Copyright (c) 2018 FIRST. All rights reserved.
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

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name = "Depot_Auto", group = "Concept")
@Disabled
public class TensorFlowView extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    int goldMineralX = -100;

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "AT0ySZn/////AAABGVDoF4gdNkZugx61WeftYqVhwz6Leeu2a1cWYQR+08xsATI6GQf3vvrvynP8JpemukCajxoFg32bkspzJx8g6uNBgHlQsFPxmFMJ8b4V/fDFTRSpy+vMOzIMoV2CHuitvtyrn/a6AsPUWczm5rsTqcCzAUEL6YD0xrzXkvaNJBzm3Jq5BkUW2ualta+LldpZ0ho/rdkDuyp6xOjsSvAbsjIDkQt807jlfYLBJAsaJNqRnQUU4mZMzl5aJsr+VnUbTfeev943zeK34ENEQzW1jCeCnLTsuWGmOd7QP+gF1YhxUr0A0s5Tr6+v8QntHPwYHp7QRpRErFqkst/OxbA4omIuLTsC41FFwYOr5YxDHhyl";
    DcMotor FrontLeft = null;
    DcMotor FrontRight = null;
    DcMotor BackLeft = null;
    DcMotor BackRight = null;

    double distPerRot = (Math.PI * 3.8125);
    double stepsPerRot = 1120;
    double lengthOfField = 12 * 12;

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private ModernRoboticsI2cGyro gyro;
    private boolean Left = false, Right = false, Center = false;

    public void Stop() {
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);
    }

    public void Forward(double power) {
        FrontLeft.setPower(power);
        FrontRight.setPower(power);
        BackLeft.setPower(power);
        BackRight.setPower(power);
    }

    public void Backward(double power) {
        FrontLeft.setPower(-power);
        FrontRight.setPower(-power);
        BackLeft.setPower(-power);
        BackRight.setPower(-power);
    }

    public void LTurn(double power) {
        FrontLeft.setPower(power);
        FrontRight.setPower(-power);
        BackLeft.setPower(power);
        BackRight.setPower(-power);
    }

    public void RTurn(double power) {
        FrontLeft.setPower(-power);
        FrontRight.setPower(power);
        BackLeft.setPower(-power);
        BackRight.setPower(power);
    }

    public void resetEncoder() {

        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
//-------

    public void moveDistance(double length) {

        resetEncoder();

        double distPerRot = Math.PI * 3.8125;
        double stepsPerRot = 560;
        double totDistInSteps = ((length / distPerRot) * stepsPerRot);

        while (totDistInSteps >= FrontLeft.getCurrentPosition() && !isStopRequested()) {

            telemetry.addData("Position", FrontLeft.getCurrentPosition());
            telemetry.addData("tot", totDistInSteps);

            telemetry.update();
            Forward(.5);

        }
    }

    public void runOpMode() {

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        runtime.reset();

        if (tfod != null) {
            tfod.activate();
        }

        FrontLeft = hardwareMap.get(DcMotor.class, "FL");
        FrontRight = hardwareMap.get(DcMotor.class, "FR");
        BackLeft = hardwareMap.get(DcMotor.class, "BL");
        BackRight = hardwareMap.get(DcMotor.class, "BR");

        gyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");

        FrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        BackRight.setDirection(DcMotorSimple.Direction.FORWARD);

        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        gyro.calibrate();


        waitForStart();

        runtime.reset();

        while (!isStopRequested()) {
            if (Left) {
                //-----------------------------------------------------
                while (!(gyro.getHeading() > 31 && gyro.getHeading() < 35)) {
                    LTurn(.2);
                }
                moveDistance(42);
                while (!(gyro.getHeading() > 313 && gyro.getHeading() < 317)) {
                    RTurn(.2);
                }
                moveDistance(30);
                gyro.resetZAxisIntegrator();
                while (!(gyro.getHeading() > 178 && gyro.getHeading() < 182)) {
                    LTurn(.2);
                }
                moveDistance(120);
                Stop();
                stop();
            } else if (Right) {
                //-----------------------------------------------------
                while (!(gyro.getHeading() > 325 && gyro.getHeading() < 329)) {
                    RTurn(.1);
                }
                moveDistance(42);
                while (!(gyro.getHeading() > 43 && gyro.getHeading() < 47)) {
                    LTurn(.1);
                }
                moveDistance(31);
                gyro.resetZAxisIntegrator();
                while (!(gyro.getHeading() > 88 && gyro.getHeading() < 92)) {
                    LTurn(.1);
                }
                moveDistance(120);
                Stop();
                stop();
            } else if (Center) {
                //-----------------------------------------------------
                moveDistance(53);
                Stop();
                while (!(gyro.getHeading() > 43 && gyro.getHeading() < 47)) {
                    LTurn(.2);
                }
                Stop();
                moveDistance(4);
                Stop();
                gyro.resetZAxisIntegrator();


                while (!(gyro.getHeading() > 88 && gyro.getHeading() < 92)) {
                    LTurn(.2);
                }
                Stop();
                moveDistance(120);
                Stop();
                stop();


            } else {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() != 0) {
                            for (Recognition recognition : updatedRecognitions) {
                                for (int i = 0; i < 15; i++) {
                                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                        goldMineralX = (int) recognition.getTop();

                                        telemetry.addData("MINEAR", goldMineralX);
                                        telemetry.update();

                                        goldMineralX = (int) recognition.getTop();
                                        telemetry.addData("MINEAR", goldMineralX);
                                        telemetry.update();

                                        goldMineralX = (int) recognition.getTop();
                                        telemetry.addData("MINEAR", goldMineralX);
                                        telemetry.update();

                                        goldMineralX = (int) recognition.getTop();
                                        telemetry.addData("MINEAR", goldMineralX);
                                        telemetry.update();
                                        if (goldMineralX > -100 && goldMineralX < 600) {
                                            telemetry.addData("Pos:", "Right");
                                            Right = true;
                                            telemetry.update();
                                        } else if (goldMineralX > +600 && goldMineralX < 1200) {
                                            telemetry.addData("Pos:", "Center");
                                            telemetry.update();
                                            Center = true;
                                        }

                                    }
                                }

                                if (Right == false && Center == false) {
                                    telemetry.addData("LEFT", "LEFT");
                                    telemetry.update();
                                    Left = true;
                                }
                            }


                        }

                    }
                    telemetry.update();
                }

            }
        }
        Stop();

        if (tfod != null)

        {
            tfod.shutdown();
        }
        Stop();

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
        parameters.cameraDirection = CameraDirection.FRONT;

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