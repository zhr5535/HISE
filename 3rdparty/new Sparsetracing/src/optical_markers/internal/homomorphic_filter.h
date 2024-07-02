#pragma once

#include<opencv2/core.hpp>

namespace dgelom {

class HighPassFilter {
public:
    /// <summary>
    /// Create a highpass filter
    /// </summary>
    /// <param name="sigma"> Parameter controls the slop of the filter</param>
    /// <param name="alpha"> Lower bound of low frequency decay factor, ranging from 0 to 1</param>
    /// <param name="beta"> Upper bound of low frequency decay factor, greater than 1</param>
    /// <returns></returns>
    HighPassFilter(float sigma=0.1, float alpha=0.25, float beta=2) noexcept;

    virtual cv::Mat make(int rows, int cols) const = 0;
    virtual ~HighPassFilter() = default;

protected:
    float sigma{ 0.1 }, alpha{ 0.25 }, beta{ 2 };
};

class GaussianHighPassFilter : public HighPassFilter 
{
public:
    cv::Mat make(int rows, int cols) const override;
};

class ButterworthHighPassFilter : public HighPassFilter 
{
private:
    int n;
public:
    ButterworthHighPassFilter(int n, float sigma = 0.1, float alpha = 0.25, float beta = 2) noexcept;
    cv::Mat make(int rows, int cols) const override;
};

cv::Mat homomorphic_filter(const cv::Mat& source,
    const HighPassFilter& hpf = GaussianHighPassFilter(), 
    int borderType = cv::BORDER_REPLICATE);

namespace internal {
    void dft_shift(cv::InputOutputArray _out);
}
}