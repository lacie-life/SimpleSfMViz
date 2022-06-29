#include "colorPrj.h"

namespace ns_clp {

    rgb trans(const hsv &val) {
        auto [h, s, v] = val;
        auto c = v * s;
        h /= 60.0;
        auto h_ = h;
        while (h_ - 2.0 >= 0.0)
            h_ -= 2.0;
        auto x = c * (1.0 - std::abs(h_ - 1));
        auto m = v - c;
        float R_ = 0.0, G_ = 0.0, B_ = 0.0;
        if (h < 1.0) {
            R_ = c;
            G_ = x;
        } else if (h < 2.0) {
            R_ = x;
            G_ = c;
        } else if (h < 3.0) {
            G_ = c;
            B_ = x;
        } else if (h < 4.0) {
            G_ = x;
            B_ = c;
        } else if (h < 5.0) {
            R_ = x;
            B_ = c;
        } else {
            R_ = c;
            B_ = x;
        }
        return rgb(int((R_ + m) * 255 + 0.5),
                   int((G_ + m) * 255 + 0.5),
                   int((B_ + m) * 255 + 0.5));
    }

    rgb project(float val, float min, float max,
                bool isReversal, int classify,
                const Color::ColorType &color) {
        if (val < min)
            val = min;
        if (val > max)
            val = max;
        auto lower = color._min;
        auto upper = color._max;
        if (isReversal)
            val = min + max - val;
        auto prj_val = (val - min) / (max - min) * (upper - lower) + lower;
        float h = 0.0, s = 0.0, v = 0.0;
        switch (color._hsv) {
        case Color::Elem::Hue: {
            if (classify > 1) {
                auto step = (upper - lower) / (classify - 1);
                h = int((prj_val - lower) / step + 0.5) * step + lower;
            } else
                h = prj_val;
            s = color._v1;
            v = color._v2;
        } break;
        case Color::Elem::Saturation: {
            if (classify > 1) {
                auto step = (upper - lower) / (classify - 1);
                s = int((prj_val - lower) / step + 0.5) * step + lower;
            } else
                s = prj_val;
            h = color._v1;
            v = color._v2;
        } break;
        case Color::Elem::Value: {
            if (classify > 1) {
                auto step = (upper - lower) / (classify - 1);
                v = int((prj_val - lower) / step + 0.5) * step + lower;
            } else
                v = prj_val;
            h = color._v1;
            s = color._v2;
        } break;
        }
        return trans(hsv(h, s, v));
    }
} // namespace ns_clp
