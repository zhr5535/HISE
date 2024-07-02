#pragma once

#include <QString>
#include <QProcess>
#if QT_VERSION < QT_VERSION_CHECK(5,15,0)
#include <QDateTime>
#else
#include <QDate>
#endif

namespace dgelom {
namespace detail {
enum class _WMIC_cmd : uint8_t
{
	GET_CPU_NAME,
	GET_CPU_ID,
	GET_CPU_CORES,
	GET_CPU_NOLPS,
	GET_BASE_SN,
	GET_BIOS_SN,
	GET_DISK_SN
};
/// <summary>
/// \brief FUNC _Get_wmic, queries hardware information with the following cmds:
/// "wmic cpu get Name"
/// "wmic cpu get NumberOfCores"
/// "wmic cpu get NumberOfLogicalProcessors"
/// "wmic cpu get processorid"
/// "wmic baseboard get serialnumber""
/// "wmic bios get serialnumber"
/// "wmic diskdrive get serialnumber"
/// </summary>
/// <param name="cmd"></param>
/// <returns></returns>
inline QString _Get_wmic(const QString& cmd) noexcept {
	QProcess _Proc;
	const auto _Cmds = cmd.split(" ");
#if QT_VERSION < QT_VERSION_CHECK(5,15,0)
	QStringList _Cmdlist;
	for (auto _It = _Cmds.begin() + 1; _It != _Cmds.end(); ++_It) {
		_Cmdlist.append(*_It);
	}
	_Proc.start(_Cmds.first(), _Cmdlist);
#else
	_Proc.start(_Cmds.first(), QStringList(_Cmds.begin()+1, _Cmds.end()));
#endif
	_Proc.waitForFinished();
	
	auto _Ret = QString::fromLocal8Bit(_Proc.readAllStandardOutput());
	_Ret = _Ret.remove(_Cmds.last(), Qt::CaseInsensitive);
	_Ret = _Ret.replace("\r", "");
	_Ret = _Ret.replace("\n", "");
	_Ret = _Ret.replace("-", "");
	_Ret = _Ret.simplified();

	return _Ret;
}

template<_WMIC_cmd _CMD>
inline QString _Get() noexcept {
	if constexpr (_CMD == _WMIC_cmd::GET_CPU_NAME) {
		return _Get_wmic("wmic cpu get name");
	}
	if constexpr (_CMD == _WMIC_cmd::GET_CPU_ID) {
		return _Get_wmic("wmic cpu get processorid");
	}
	if constexpr (_CMD == _WMIC_cmd::GET_CPU_CORES) {
		return _Get_wmic("wmic cpu get numberofcores");
	}
	if constexpr (_CMD == _WMIC_cmd::GET_CPU_NOLPS) {
		return _Get_wmic("wmic cpu get NumberOfLogicalProcessors");
	}
	if constexpr (_CMD == _WMIC_cmd::GET_BASE_SN) {
		return _Get_wmic("wmic baseboard get serialnumber");
	}
	if constexpr (_CMD == _WMIC_cmd::GET_BIOS_SN) {
		return _Get_wmic("wmic bios get serialnumber");
	}
	if constexpr (_CMD == _WMIC_cmd::GET_DISK_SN) {
		return _Get_wmic("wmic diskdrive where index=0 get serialnumber");
	}
}
}
inline QString make_identifier_code() noexcept {
	using CMD = detail::_WMIC_cmd;
	return detail::_Get<CMD::GET_BASE_SN>().right(4) + "-"
		+ detail::_Get<CMD::GET_BIOS_SN>().left(4) + "-"
		+ detail::_Get<CMD::GET_CPU_ID>().right(4) + "-"
		+ detail::_Get<CMD::GET_DISK_SN>().left(4);
}

inline bool is_expired(QDate&& date) noexcept
{
	const auto __dvalue = [](size_t y, size_t m, size_t d) {
		size_t mon[] = { 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334 };
		return y * 365 + y / 4 - y / 100 + y / 400 + mon[m - 1] + d - 1 + (((y % 100 != 0 && y % 4 == 0) || y % 400 == 0) && m > 2);
	};

	const auto cur = QDate::currentDate();
	return __dvalue(cur.year(), cur.month(), cur.day()) > __dvalue(date.year(), date.month(), date.day());
}
}