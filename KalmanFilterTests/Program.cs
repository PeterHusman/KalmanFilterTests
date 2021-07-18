using KalmanFilters;

using MathNet.Numerics.Distributions;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

using OxyPlot;
using OxyPlot.Axes;
using OxyPlot.Series;

using System;
using System.Diagnostics;
using System.IO;

namespace KalmanFilterTests
{
    class Program
    {
        static Normal normalDistribution;
        static IKalmanFilter filter;

        static void Main(string[] args)
        {
            double deltaT = 0.05;



            Func<Vector, Matrix, (Vector<double> sigmaPoint, double weight_m, double weight_c)[]> getSigs;
            //getSigs = UnscentedKalmanFilter.GetGetSigmaPointsMethodOne(1e-3, 2, 0);
            getSigs = UnscentedKalmanFilter.GetGetSigmaPointsMethodTwo(0);


            Matrix stateTransition = DenseMatrix.OfArray(new double[,]
            {
                { 1,                        0,      0 },
                { deltaT,                   1,      0 },
                { 0.5 * deltaT * deltaT,    deltaT, 1 }
            });

            double[] initialState = new double[] { 0, 0, 0 };

            double measurementCovariance = 100;

            Matrix observationModel = DenseMatrix.OfArray(new double[,]
            {
                { 0, 0, 1 }
            });

            double[] controlModel = new double[]
            {
                0,
                0,
                deltaT
            };

            Matrix processCovariance = DenseMatrix.OfArray(new double[,]
            {
                { 0.01,   0,      0 },
                { 0,      0.01,   0 },
                { 0,      0,      0.01 }
            });

            Matrix initialCovariance = DenseMatrix.OfArray(new double[,]
            {
                { 100, 0, 0 },
                { 0, 100, 0 },
                { 0, 0, 100 }
            });


            PlotModel plot = new PlotModel() { Title = "Position estimate with time" };

            LineSeries positionEstimateSeries = new LineSeries{};

            LinearAxis yAxis = new LinearAxis
            {
                Minimum = 0,
                Position = AxisPosition.Left
            };
            LinearAxis xAxis = new LinearAxis
            {
                Minimum = 0,
                Position = AxisPosition.Bottom
            };
            plot.Axes.Add(yAxis);
            plot.Axes.Add(xAxis);
            plot.Series.Add(positionEstimateSeries);

            //filter = new KalmanFilter((Matrix)new DenseMatrix(3, 3, stateTransition).Transpose(), (Matrix)new DenseMatrix(3, 3, processCovariance).Transpose(), DenseMatrix.Create(1, 1, measurementCovariance), (Matrix)new DenseMatrix(3, 1, observationModel).Transpose(), (Matrix)new DenseMatrix(1, 3, controlModel).Transpose(), new DenseVector(initialState), (Matrix)new DenseMatrix(3, 3, initialCovariance).Transpose()/*DenseMatrix.Create(3, 3, 100)*/);

            filter = new UnscentedKalmanFilter(a => (Vector)(stateTransition * a), processCovariance, a => (Vector)(observationModel * a), DenseMatrix.Create(1, 1, measurementCovariance), new DenseVector(initialState), initialCovariance, getSigs);

            normalDistribution = new Normal(0, Math.Sqrt(measurementCovariance));

            double position = 100;
            double velocity = 0;
            double acceleration = 0;

            Stopwatch watch = new Stopwatch();

            watch.Start();

            double simTime = 0;

            while (/*true || */simTime < 200)
            {
                positionEstimateSeries.Points.Add(new DataPoint(simTime, filter.StateEstimate[2]));

                simTime += deltaT;
                position += velocity * deltaT;
                velocity += acceleration * deltaT;
                position += 0.5 * deltaT * deltaT * acceleration;

                filter.Predict();

                double reading = position + normalDistribution.Sample();
                filter.Update(new DenseVector(new double[] { reading }));


                Console.WriteLine($"Pos: {position:0.00}\t\tEstimate: {filter.StateEstimate[2]:0.00} {filter.StateEstimate[1]:0.00} {filter.StateEstimate[0]:0.00}\t\tVariances: {filter.EstimateCovariance[2, 2]:0.00} {filter.EstimateCovariance[1, 1]:0.00} {filter.EstimateCovariance[0, 0]:0.00}\t\tReading: {reading:0.00}");

                //acceleration += 0.3;
                while (false && watch.Elapsed.TotalSeconds < deltaT)
                {

                }
                watch.Restart();
            }

            using FileStream stream = File.Create("myPdf.pdf");
            var exporter = new PdfExporter() { Width = 600, Height = 400 };
            exporter.Export(plot, stream);
        }
    }
}
