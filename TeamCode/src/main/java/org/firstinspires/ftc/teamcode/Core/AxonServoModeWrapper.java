package org.firstinspires.ftc.teamcode.Core;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class AxonServoModeWrapper {

    private final Servo axon;
    private final AnalogInput encoder;

    private double lastReadPosition;

    private final boolean inversePosition;
    private final boolean inverseEncoder;

    private double encoderOffset;

    public AxonServoModeWrapper(
            Servo axon,
            AnalogInput encoder,
            boolean inversePosition,
            boolean inverseEncoder,
            double encoderOffset
    ) {

        this.axon = axon;
        this.encoder = encoder;

        this.inversePosition = inversePosition;
        this.inverseEncoder = inverseEncoder;

        this.encoderOffset = encoderOffset;
    }

    // Command Axon in servo mode: 0.0 to 1.0
    public void setPosition(double position) {

        position = Range.clip(position, 0.0, 1.0);

        if (inversePosition) {
            position = 1.0 - position;
        }

        axon.setPosition(position);
    }

    public double getPosition() {

        double position = axon.getPosition();

        if (inversePosition) {
            position = 1.0 - position;
        }

        return position;
    }

    public void readPos() {

        double angle =
                encoder.getVoltage()
                        / 3.3
                        * 360.0;

        angle += encoderOffset;

        if (inverseEncoder) {
            angle = 360.0 - angle;
        }

        angle %= 360.0;

        if (angle < 0) {
            angle += 360.0;
        }

        lastReadPosition = angle;
    }

    public double readAndGetVoltage() {
        return encoder.getVoltage();
    }

    public double readAndGetRawPos() {
        return encoder.getVoltage() / 3.3;
    }

    public double getServoAngle() {
        return lastReadPosition;
    }

    public void setEncoderOffset(double offset) {
        encoderOffset = offset;
    }
}