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

@Autonomous(name = "oiahr", group = "Concept")
//@Disabled
public class ConceptTensorFlowObjectDetection extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = " AT0ySZn/////AAABGVDoF4gdNkZugx61WeftYqVhwz6Leeu2a1cWYQR+08xsATI6GQf3vvrvynP8JpemukCajxoFg32bkspzJx8g6uNBgHlQsFPxmFMJ8b4V/fDFTRSpy+vMOzIMoV2CHuitvtyrn/a6AsPUWczm5rsTqcCzAUEL6YD0xrzXkvaNJBzm3Jq5BkUW2ualta+LldpZ0ho/rdkDuyp6xOjsSvAbsjIDkQt807jlfYLBJAsaJNqRnQUU4mZMzl5aJsr+VnUbTfeev943zeK34ENEQzW1jCeCnLTsuWGmOd7QP+gF1YhxUr0A0s5Tr6+v8QntHPwYHp7QRpRErFqkst/OxbA4omIuLTsC41FFwYOr5YxDHhyl";
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

    public void RTurn(double power) {
        FrontLeft.setPower(-power);
        FrontRight.setPower(power);
        BackLeft.setPower(-power);
        BackRight.setPower(power);
    }

    public void LTurn(double power) {
        FrontLeft.setPower(power);
        FrontRight.setPower(-power);
        BackLeft.setPower(power);
        BackRight.setPower(-power);
    }

    public void moveDistance(double length) {

        double distPerRot = Math.PI * 3.8125;
        double stepsPerRot = 1120;
        double totDistInSteps = ((length / distPerRot) * stepsPerRot);

        double step = FrontLeft.getCurrentPosition();


        if (totDistInSteps < (step)) {
            while (totDistInSteps <= (step)) {

                telemetry.addData("Position", FrontLeft.getCurrentPosition());
                telemetry.addData("tot", totDistInSteps);

                telemetry.update();
                Backward(.5);

            }
        } else {
            Stop();
        }
    }


    @Override
    public void runOpMode() {

        runtime.milliseconds();

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
        waitForStart();

        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
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
            BackRight.setDirection(DcMotorSimple.Direction.REVERSE);

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

            while (opModeIsActive()) {
                runtime.reset();

                if (tfod != null) {

                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        int goldMineralX = -100;

                        if (updatedRecognitions.size() != 0) {
                            double top = 0;
                            double left = 0;
                            double right = 0;
                            double bottom = 0;

                            for (Recognition recognition : updatedRecognitions) {
                                for (int i = 0; i < 5 && !isStopRequested(); i++) {
                                    sleep(200);//--------------------------------

                                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                        if (recognition.getLeft() >= 130) {
                                            telemetry.addData("dasdasfasfsdf", "I see it");
                                            top = recognition.getTop();
                                            left = recognition.getLeft();
                                            right = recognition.getRight();
                                            bottom = recognition.getBottom();

                                            telemetry.addData("top", top);
                                            telemetry.addData("left", left);
                                            telemetry.addData("right", right);
                                            telemetry.addData("bottom", bottom);
                                            telemetry.update();


                                            if (top > -100 && top < 600) {
                                                Right = true;
                                            } else if (top > 600 && top < 1200) {
                                                Center = true;
                                            }
                                        } else {
                                            telemetry.addData("No Soo Anthing", "wser");
                                        }
                                    }

                                    if (Right == false && Center == false) {
                                        Left = true;
                                    }


                                }

                                telemetry.clearAll();
                                telemetry.addData("top", top);
                                telemetry.addData("left", left);
                                telemetry.addData("right", right);
                                telemetry.addData("bottom", bottom);
                                telemetry.addData("left", Left);
                                telemetry.addData("right", Right);
                                telemetry.addData("center", Center);
                                telemetry.update();
                            }
                        }


                    }
                }
            }
        }


        if (tfod != null) {
            tfod.shutdown();
        }
    }


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


//ethan.hampton@msd.oregonk-12.net
//NHS-Programming@hotmail.com