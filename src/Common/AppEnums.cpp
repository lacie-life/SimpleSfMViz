#include "AppEnums.h"

QMap<int, QString> AppEnums::MODEL_ZOO = QMap<int, QString>({{AppEnums::DETECT_MODEL::YOLO, "YoLo"},
                                                             {AppEnums::DETECT_MODEL::SSD, "SSD"},
                                                             {AppEnums::DETECT_MODEL::MODEL_MAX, "MAX"}});

