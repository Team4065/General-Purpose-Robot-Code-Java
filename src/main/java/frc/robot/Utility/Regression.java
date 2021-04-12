// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utility;

import java.util.Arrays;
import java.util.Vector;
import java.util.function.BiFunction;

/** Add your docs here. */
public class Regression {

    static private Double[] addDirection(Double[] explainingVariables, Double[] direction){
        Double[] output = new Double[explainingVariables.length];

        int index = 0;
        for(Double value : explainingVariables){
            output[index] = value + direction[index];
            ++index;
        }

        return output;
    }

    static private Double[] getDirection(int combinationIndex, double intensity, int digits){
        Vector<Double> tempOutput = new Vector<Double>();

        for(int i = 0; i < digits; ++i){
            int base3 = combinationIndex % 3;
            combinationIndex = combinationIndex / 3;
            tempOutput.add((double)(base3 - 1) * intensity);
        }

        Double[] output = new Double[tempOutput.size()];
        tempOutput.toArray(output);
        return output;
    }

    static private double getError(BiFunction<Double[], Double[], Double> func, Double[][] independentVariablesRecording, Double[] dependentVariableRecording, Double[] explainingVariables){
        double error = 0;

        for(int i = 0; i < independentVariablesRecording.length; ++i){
            error += Math.pow(dependentVariableRecording[i] - func.apply(independentVariablesRecording[i], explainingVariables), 2);
        }

        return error;
    }

    static public Double[] performRegression(int numberOfIndependentVariables, int numberOfExplainingVariables, BiFunction<Double[], Double[], Double> func, Double[][] independentVariablesRecording, Double[] dependentVariableRecording){
        Double[] explainingVariables = new Double[numberOfExplainingVariables];
        Arrays.fill(explainingVariables, 0.0);
        int possibleCombinations = (int)Math.pow(3, numberOfExplainingVariables);

        double intensity = 1;
        for(int j = 0; j < 1000; ++j){

            int lowestCombination = 0;
            double lowestError = Double.POSITIVE_INFINITY;

            for(int i = 0; i < possibleCombinations; ++i){
                Double[] variables = Regression.addDirection(explainingVariables, Regression.getDirection(i, intensity, numberOfExplainingVariables));

                double error = Regression.getError(func, independentVariablesRecording, dependentVariableRecording, variables);

                if(error < lowestError){
                    lowestCombination = i;
                    lowestError = error;
                }
            }

            explainingVariables = Regression.addDirection(explainingVariables, Regression.getDirection(lowestCombination, intensity, numberOfExplainingVariables));
            intensity *= 0.99;
        }
        
        return explainingVariables;
    }

    static public Double[] findFeedForwardGainsForVelocity(Vector<Double> controlledVariableRecording, Vector<Double> velocityRecording, Vector<Double> accelerationRecording){
        BiFunction<Double[], Double[], Double> func = (independentVariables, explainingVariables)->{
            return explainingVariables[0] * Math.signum(independentVariables[0]) + explainingVariables[1] * independentVariables[0] + explainingVariables[2] * independentVariables[1];
        };

        Double[][] independentVariables = new Double[velocityRecording.size()][2];
        for(int i = 0; i < velocityRecording.size(); ++i){
            independentVariables[i][0] = velocityRecording.elementAt(i);
            independentVariables[i][1] = accelerationRecording.elementAt(i);
        }
        Double[] dependentVariable = new Double[velocityRecording.size()];
        controlledVariableRecording.toArray(dependentVariable);

        return Regression.performRegression(2, 3, func, independentVariables, dependentVariable);
    }

}
