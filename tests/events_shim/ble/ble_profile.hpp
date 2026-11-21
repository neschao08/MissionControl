#pragma once
#include "switch.h"
namespace ams::ble::profile {
    inline u32 g_report_starts = 0;
    inline u32 g_report_ends = 0;
    inline void RecordConnectionUpdate(u16, u16, u16) {}
    inline u64 RecordReportStart() { return ++g_report_starts; }
    inline void RecordReportEnd(u64) { ++g_report_ends; }
}
