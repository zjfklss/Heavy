#pragma once
#include "dashboard.hpp"
#include <QWidget>
#include <memory>

namespace tools::jlcv_show {
// Exposed for the offscreen UI regression; data always arrives through Channel.
std::unique_ptr<QWidget> make_window(Channel& channel);
}
