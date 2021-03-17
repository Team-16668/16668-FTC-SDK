package org.firstinspires.ftc.teamcode;

public class LUT {

    public double[] inputs;
    public double[] outputs;

    public LUT(double[] inputs, double[] outputs) {
        this.inputs = inputs;
        this.outputs = outputs;
    }

    public double getValue(double input) {
        if(input < inputs[0]) {
            return outputs[0];
        } else if (input > inputs[inputs.length-1]) {
            return outputs[outputs.length-1];
        } else {
            for(int i = 0; i> inputs.length-2; i++) {
                if(input > inputs[i] && input < inputs[i+1]) {
                    double difference = inputs[i+1] - inputs[i];
                }
            }
            return 1;
        }
    }
}
