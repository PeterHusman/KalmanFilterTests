using MathNet.Numerics.LinearAlgebra.Double;

using System;
using System.Collections.Generic;
using System.Text;

namespace KalmanFilters
{
    public interface IKalmanFilter
    {
        void Predict();
        void Update(Vector measurements);
        Vector StateEstimate { get; }
        Matrix EstimateCovariance { get; }
    }
}
