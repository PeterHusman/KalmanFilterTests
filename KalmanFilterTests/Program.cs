﻿using KalmanFilters;

using MathNet.Numerics.Distributions;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

using System;
using System.Diagnostics;

namespace KalmanFilterTests
{
    class Program
    {
        static Normal normalDistribution;
        static IKalmanFilter filter;

        static void Main(string[] args)
        {
            double deltaT = 0.05;
            double[] stateTransition = new[]
            {
                1,      0,      0,
                deltaT, 1,      0,
                0.5 * deltaT * deltaT,  deltaT, 1
            };

#if true
            Matrix covMat = new DenseMatrix(2, 2, new double[] { 2, 1, 1, 3 });
            // a[0] = 1 * a[0] + 0 * a[1]
            // a[1] = 0.5 * a[0] + 1 * a[1]
            Matrix sta = (Matrix)new DenseMatrix(2, 2, new double[] { 1, 0.5, 0, 1 });

            Vector val = new DenseVector(new double[] { 1, 0 });

            var newCovMat = sta * covMat * sta.Transpose();
            var newMean = sta * val;

            Func<Vector, Matrix, (Vector<double> sigmaPoint, double weight_m, double weight_c)[]> getSigs;
            //getSigs = UnscentedKalmanFilter.GetGetSigmaPointsMethodOne(1e-3, 2, 0);
            getSigs = UnscentedKalmanFilter.GetGetSigmaPointsMethodTwo(1);

            var sigmas = getSigs(val, /*new DenseMatrix(2, 2, new double[] { 2, 1, 1, 3 })*/covMat);
            for (int i = 0; i < sigmas.Length; i++)
            {
                sigmas[i].sigmaPoint = sta * sigmas[i].sigmaPoint;// (Vector<double>)new DenseVector(new double[] { sigmas[i].sigmaPoint[0], sigmas[i].sigmaPoint[1] + sigmas[i].sigmaPoint[0] * 0.5 });
            }
            var transformed = UnscentedKalmanFilter.UnscentedTransform(sigmas, new DenseMatrix(2));
#endif

            Console.ReadLine();
            

            double[] processNoise = new[] { 0.1, 0.3, 0.5 };

            double[] initialState = new[] { 0, 0, 100.0 };

            //double[] initialCovariance = new[] { 1.0, 100, 100 };

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

            double[] processCovariance = new double[]
            {
                10, 0, 1,
                0, 100, 1,
                1, 1, 400
            };

            // What does 'positive definite' mean?
            // HOW did this end up giving me a negative variance?
            // Something is deeply broken.
            // F I X   I T .
            /*processCovariance*/_ = new double[]
            {
                0.1, 0, 1,
                0, 1, 1,
                1, 1, 5
            };

            //processCovariance = new double[] { 0, 0, 0, 0, 0, 0, 0, 0, 0 };

            double[] initialCovariance = new double[]
            {
                1, 0, 0,
                0, 2, 0,
                0, 0, 100
            };

            // What does this do?
            double kappa = 1;

            //filter = new KalmanFilter((Matrix)new DenseMatrix(3, 3, stateTransition).Transpose(), (Matrix)new DenseMatrix(3, 3, processCovariance).Transpose(), DenseMatrix.Create(1, 1, measurementCovariance), (Matrix)new DenseMatrix(3, 1, observationModel).Transpose(), (Matrix)new DenseMatrix(1, 3, controlModel).Transpose(), new DenseVector(initialState), (Matrix)new DenseMatrix(3, 3, initialCovariance).Transpose()/*DenseMatrix.Create(3, 3, 100)*/);

            filter = new UnscentedKalmanFilter(a => new DenseVector(new[] { a[0], a[1] + deltaT * a[0], a[2] + deltaT * a[1] + 0.5 * deltaT * deltaT * a[0] }), (Matrix)new DenseMatrix(3, 3, processCovariance).Transpose(), a => new DenseVector(new[] { a[2] }), DenseMatrix.Create(1, 1, measurementCovariance), new DenseVector(initialState), (Matrix)new DenseMatrix(3, 3, initialCovariance).Transpose(), getSigs);

            normalDistribution = new Normal(0, Math.Sqrt(measurementCovariance));

            double position = 100;
            double velocity = 0;
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
                Console.WriteLine($"Pos: {position:0.00}\t\tEstimate: {filter.StateEstimate[2]:0.00} {filter.StateEstimate[1]:0.00} {filter.StateEstimate[0]:0.00}\t\tVariances: {filter.EstimateCovariance[2, 2]:0.00} {filter.EstimateCovariance[1, 1]:0.00} {filter.EstimateCovariance[0, 0]:0.00}\t\tReading: {reading:0.00}");

                acceleration += 0.3;
                while (false && watch.Elapsed.TotalSeconds < deltaT)
                {

                }
                watch.Restart();
            }
        }
    }
}
