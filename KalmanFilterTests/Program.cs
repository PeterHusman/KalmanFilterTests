using KalmanFilters;

using MathNet.Numerics.Distributions;
using MathNet.Numerics.LinearAlgebra.Double;

using System;
using System.Diagnostics;

namespace KalmanFilterTests
{
    class Program
    {
        static Normal normalDistribution;
        static KalmanFilter filter;

        static void Main(string[] args)
        {
            double deltaT = 0.05;
            double[] stateTransition = new[]
            {
                1,      0,      0,
                deltaT, 1,      0,
                1/2 * deltaT * deltaT,  deltaT, 1
            };

            double[] processNoise = new[] { 0.1, 0.3, 0.5 };

            double[] initialState = new[] { 0, 0, 0.0 };

            double[] initialCovariance = new[] { 100.0, 100, 100 };

            double measurementCovariance = 100;

            double[] observationModel = new double[]
            {
                0, 0, 1
            };

            double[] controlModel = new double[]
            {
                0,
                0,
                deltaT
            };

            // So
            // I thought covariance was just gonna be a vector.
            // Turns out it's an n x n matrix.
            // Why??
            // Figure this out next.

            filter = new KalmanFilter((Matrix)new DenseMatrix(3, 3, stateTransition).Transpose(), DenseMatrix.Create(3, 3, 0.01), DenseMatrix.Create(1, 1, measurementCovariance), (Matrix)new DenseMatrix(3, 1, observationModel).Transpose(), (Matrix)new DenseMatrix(1, 3, controlModel).Transpose(), new DenseVector(initialState), DenseMatrix.Create(3, 3, 100));

            normalDistribution = new Normal(0, Math.Sqrt(measurementCovariance));

            double position = 0;
            double velocity = 5;
            double acceleration = 0;

            Stopwatch watch = new Stopwatch();

            watch.Start();

            while (true)
            {
                position += velocity * deltaT;
                position += 1 / 2 * deltaT * deltaT * acceleration;

                filter.Predict();

                double reading = position + normalDistribution.Sample();
                filter.Update(new DenseVector(new double[] { reading }));
                Console.WriteLine($"Pos: {position:0.00}\t\tEstimate: {filter.StateEstimate[2]:0.00} {filter.StateEstimate[1]:0.00} {filter.StateEstimate[0]:0.00}\t\tReading: {reading}");

                while (watch.Elapsed.TotalSeconds < deltaT)
                {

                }
                watch.Restart();
            }
        }
    }
}
