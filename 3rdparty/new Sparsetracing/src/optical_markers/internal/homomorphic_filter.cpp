#include "homomorphic_filter.h"

namespace dgelom {
HighPassFilter::HighPassFilter(float sigma, float alpha, float beta) noexcept
{
    this->alpha = alpha;
    this->beta = beta;
    this->sigma = sigma;
}
cv::Mat GaussianHighPassFilter::make(int rows, int cols) const {
    cv::Mat filter = cv::Mat(rows, cols, CV_32FC1);
    const float d0 = 2 * powf(sigma, 2);
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            // D(i, j)^2
            float numerator = powf((i - rows / 2.0f), 2.0) + powf((j - cols / 2.0f), 2.0);
            // H(i, j)
            float value = (1.0f - expf(-1 * numerator / d0));
            // high-freq emphasis filter (alpha, beta)
            filter.at<float>(i, j) = (beta - alpha) * value + alpha;
        }
    }
    return filter;
}

ButterworthHighPassFilter::ButterworthHighPassFilter(int n, float sigma, float alpha, float beta) noexcept
    : HighPassFilter(sigma, alpha, beta) {
    this->n = n;
}

cv::Mat ButterworthHighPassFilter::make(int rows, int cols) const {
    cv::Mat filter = cv::Mat(rows, cols, CV_32FC1);
    const float d0 = 2 * powf(sigma, 2);

    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            // D(i, j)^2
            float numerator = powf(i - rows / 2.0f, 2.0) + powf(j - cols / 2.0f, 2.0);
            // D(i, j)
            numerator = powf(numerator, 0.5);
            // H(i, j)
            float value = 1.0 / (1 + powf(d0 / numerator, 2 * this->n));
            // high-freq emphasis filter (alpha, beta)
            filter.at<float>(i, j) = (beta - alpha) * value + alpha;
        }
    }
    return filter;
}

cv::Mat homomorphic_filter(const cv::Mat& source, const HighPassFilter& hpf, int borderType) 
{
    // convert to 32f, normalize and log (1 + x) - add 1 to remove 0s
    cv::Mat log;
    source.convertTo(log, CV_32F);
    log /= 255.0;
    //cv::normalize(log, log, 0, 1, cv::NORM_MINMAX);
    log += cv::Scalar::all(1);
    cv::log(log, log);

    // padding to optimal fft size
    cv::Mat padded;
    const int m = cv::getOptimalDFTSize(source.rows);
    const int n = cv::getOptimalDFTSize(source.cols);
    const int isXodd = (source.cols % 2) == 1;
    const int isYodd = (source.rows % 2) == 1;
    const int hpad = (n - source.cols) / 2;
    const int vpad = (m - source.rows) / 2;
    cv::copyMakeBorder(log, padded, vpad, vpad + isYodd, hpad, hpad + isXodd, borderType);

    // high pass filter
    cv::Mat highPass = hpf.make(padded.rows, padded.cols);

    // move filter to the center
    internal::dft_shift(highPass);

    // frequency domain filtering
    cv::Mat planes[] = { padded, cv::Mat::zeros(padded.size(), CV_32FC1) };
    cv::Mat complex;
    cv::merge(planes, 2, complex);
    cv::dft(complex, complex);
    cv::split(complex, planes);

    // filtering and iFFT
    cv::Mat processedReal = cv::Mat(planes[0].size(), planes[0].type());
    cv::Mat processedComplex = cv::Mat(planes[1].size(), planes[1].type());
    cv::multiply(planes[0], highPass, processedReal);
    cv::multiply(planes[1], highPass, processedComplex);
    cv::Mat processed[] = { processedReal, processedComplex };
    cv::merge(processed, 2, complex);

    // inverse dft and get real
    cv::Mat output;
    cv::idft(complex, complex);
    cv::extractChannel(complex, output, 0);
    output = cv::Mat(output, cv::Rect(hpad, vpad, source.cols, source.rows));

    // apply expr(I) - 1
    cv::normalize(output, output, 0, 1, cv::NORM_MINMAX);
    cv::exp(output, output);
    output -= 1.0;

    // image denormalization
    cv::normalize(output, output, 0, 255, cv::NORM_MINMAX);
    cv::Mat dest;
    output.convertTo(dest, CV_8UC1);
    return dest;
}

void internal::dft_shift(cv::InputOutputArray _out) {
    cv::Mat out = _out.getMat();

    if (out.rows == 1 && out.cols == 1) {
        // trivially shifted.
        return;
    }

    std::vector<cv::Mat> planes;
    split(out, planes);

    int xMid = out.cols >> 1;
    int yMid = out.rows >> 1;

    bool is_1d = xMid == 0 || yMid == 0;

    if (is_1d) {
        const int is_odd = (xMid > 0 && out.cols % 2 == 1) || (yMid > 0 && out.rows % 2 == 1);
        xMid = xMid + yMid;

        for (size_t i = 0; i < planes.size(); i++) {
            cv::Mat half0(planes[i], cv::Rect(0, 0, xMid + is_odd, 1));
            cv::Mat half1(planes[i], cv::Rect(xMid + is_odd, 0, xMid, 1));

            cv::Mat tmp;
            half0.copyTo(tmp);
            half1.copyTo(planes[i](cv::Rect(0, 0, xMid, 1)));
            tmp.copyTo(planes[i](cv::Rect(xMid, 0, xMid + is_odd, 1)));
        }
    }
    else {
        const int isXodd = out.cols % 2 == 1;
        const int isYodd = out.rows % 2 == 1;
        for (size_t i = 0; i < planes.size(); i++) {
            // perform quadrant swaps...
            cv::Mat q0(planes[i], cv::Rect(0, 0, xMid + isXodd, yMid + isYodd));
            cv::Mat q1(planes[i], cv::Rect(xMid + isXodd, 0, xMid, yMid + isYodd));
            cv::Mat q2(planes[i], cv::Rect(0, yMid + isYodd, xMid + isXodd, yMid));
            cv::Mat q3(planes[i], cv::Rect(xMid + isXodd, yMid + isYodd, xMid, yMid));

            if (!(isXodd || isYodd)) {
               /* cv::Mat tmp;
                q0.copyTo(tmp);
                q3.copyTo(q0);
                tmp.copyTo(q3);*/
                cv::swap(q0, q3);

                /*q1.copyTo(tmp);
                q2.copyTo(q1);
                tmp.copyTo(q2);*/
                cv::swap(q1, q2);
            }
            else {
                cv::Mat tmp0, tmp1, tmp2, tmp3;
                q0.copyTo(tmp0);
                q1.copyTo(tmp1);
                q2.copyTo(tmp2);
                q3.copyTo(tmp3);

                tmp0.copyTo(planes[i](cv::Rect(xMid, yMid, xMid + isXodd, yMid + isYodd)));
                tmp3.copyTo(planes[i](cv::Rect(0, 0, xMid, yMid)));

                tmp1.copyTo(planes[i](cv::Rect(0, yMid, xMid, yMid + isYodd)));
                tmp2.copyTo(planes[i](cv::Rect(xMid, 0, xMid + isXodd, yMid)));
            }
        }
    }
    cv::merge(planes, out);
}
}