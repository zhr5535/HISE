#pragma once
#include <QString>
#include <opencv2/core.hpp>
#include <opencv2/core/core_c.h>
#include <opencv2/imgcodecs.hpp>
#include <type_traits>

namespace {
using cvmat_ptr = std::add_pointer_t<CvMat>;
using const_cvmat_ptr = const CvMat*;

template<typename T> static constexpr auto is_uint8_v = std::is_same_v<T, uint8_t>;
template<typename T> static constexpr auto is_double_v = std::is_same_v<T, double>;
template<typename T> static constexpr auto is_float_v = std::is_same_v<T, float>;

cvmat_ptr read_cvmat_64f(const std::string& path) {
	auto mat = cv::imread(path, cv::IMREAD_GRAYSCALE);
	if (mat.empty()) {
		return nullptr;
	}
	auto cvmat_1 = cvMat(mat);
	auto cvmat = cvCreateMat(mat.rows, mat.cols, CV_64F);
	cvConvertScale(&cvmat_1, cvmat);
	return cvmat;
}

template<typename T>
cvmat_ptr read_cvmat(const QString& path) {
	const auto mat = cv::imread(path.toStdString(), cv::IMREAD_GRAYSCALE);
	if (mat.empty()) {
		return nullptr;
	}
	auto cvmat = cvMat(mat);
	if constexpr (std::is_same_v<T, double>) {
		auto _Ret = cvCreateMat(mat.rows, mat.cols, CV_64F);
		cvConvertScale(&cvmat, _Ret);
		return _Ret;
	}
	else if constexpr (std::is_same_v<T, float>) {
		auto _Ret = cvCreateMat(mat.rows, mat.cols, CV_32F);
		cvConvertScale(&cvmat, _Ret);
		return _Ret;
	}
	else {
		auto _Ret = cvCreateMat(mat.rows, mat.cols, CV_8U);
		cvCopy(&cvmat, _Ret);
		return _Ret;
	}
}
cvmat_ptr read_cvmat_64f(const std::string& path, cvmat_ptr dst) {
	auto mat = cv::imread(path, cv::IMREAD_GRAYSCALE);
	if (mat.empty()) {
		return nullptr;
	}
	auto cvmat_1 = cvMat(mat);
	cvConvertScale(&cvmat_1, dst);
	return dst;
}
template<typename T>
cvmat_ptr read_cvmat(const QString& path, cvmat_ptr _Ret) {
	const auto mat = cv::imread(path.toStdString(), cv::IMREAD_GRAYSCALE);
	if (mat.empty()) {
		return nullptr;
	}
	auto cvmat = cvMat(mat);
	if constexpr (is_double_v<T>) {
		cvConvertScale(&cvmat, _Ret);
		return _Ret;
	}
	else if constexpr (is_float_v<T>) {
		cvConvertScale(&cvmat, _Ret);
		return _Ret;
	}
	else {
		static_assert(is_uint8_v<T>, "The type T in read_cvmat<T>() should be uint8_t or uchar.");
		cvCopy(&cvmat, _Ret);
		return _Ret;
	}
}

cv::Mat get_mat(cvmat_ptr mat) {
	return cv::Mat(mat->rows, mat->cols, mat->type, mat->data.ptr, mat->step);
}
cv::Mat get_mat(const_cvmat_ptr mat) {
	return cv::Mat(mat->rows, mat->cols, mat->type, mat->data.ptr, mat->step);
}
}