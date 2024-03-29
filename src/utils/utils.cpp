//
// Created by jerry on 23-4-28.
//

#include "utils/utils.h"


int showMessageBox(QWidget *p, STATE state, QString text, int btn_num, QVector<QString> btn_text) {
    auto box = new QMessageBox(p);
    box->setWindowFlags(Qt::FramelessWindowHint | Qt::Dialog);
    box->setText(text);
    QImage qimage;
    switch (state) {
        case SUCCESS:
            qimage = QImage(":m_icon/icon/confirm.png");
            break;
        case WARN:
            qimage = QImage(":m_icon/icon/warn.png");
            break;
        case ERROR:
            qimage = QImage(":m_icon/icon/dididi.png");
            break;
    }
    qimage = qimage.scaled(50, 50, Qt::AspectRatioMode::KeepAspectRatio, Qt::TransformationMode::SmoothTransformation);
    box->setIconPixmap(QPixmap::fromImage(qimage));
//    box->addButton()
    for (int i = 0; i < btn_num; ++i) {
        box->addButton(btn_text[i], QMessageBox::ActionRole);
    }
    box->show();
    return box->exec();
}


unsigned long getTickCount() {
    struct timeval tv;
    if (gettimeofday(&tv, NULL) != 0)
        return 0;
    return (tv.tv_sec * 1000) + (tv.tv_usec / 1000);
}


bool isFastClick(QObject *target, int delayTimeMil) {
    qlonglong lastTick = (target->property("tick").toLongLong());
    qlonglong tick = getTickCount();
    target->setProperty("tick", tick);
    if (tick - lastTick > delayTimeMil) {
        return true;
    }
    return false;
}

QImage cvMat2QImage(const cv::Mat &mat) {
    // 8-bits unsigned, NO. OF CHANNELS = 1
    if (mat.type() == CV_8UC1) {
        QImage image(mat.cols, mat.rows, QImage::Format_Indexed8);
        // Set the color table (used to translate colour indexes to qRgb values)
        image.setColorCount(256);
        for (int i = 0; i < 256; i++) {
            image.setColor(i, qRgb(i, i, i));
        }
        // Copy input Mat
        uchar *pSrc = mat.data;
        for (int row = 0; row < mat.rows; row++) {
            uchar *pDest = image.scanLine(row);
            memcpy(pDest, pSrc, mat.cols);
            pSrc += mat.step;
        }
        return image;
    }
        // 8-bits unsigned, NO. OF CHANNELS = 3
    else if (mat.type() == CV_8UC3) {
        // Copy input Mat
        const uchar *pSrc = (const uchar *) mat.data;
        // Create QImage with same dimensions as input Mat
        QImage image(pSrc, mat.cols, mat.rows, mat.step, QImage::Format_RGB888);
        return image.rgbSwapped();
    } else if (mat.type() == CV_8UC4) {
        qDebug("CV_8UC4");
        // Copy input Mat
        const uchar *pSrc = (const uchar *) mat.data;
        // Create QImage with same dimensions as input Mat
        QImage image(pSrc, mat.cols, mat.rows, mat.step, QImage::Format_ARGB32);
        return image.copy();
    } else {
        qDebug("ERROR: Mat could not be converted to QImage.");
        return QImage();
    }
}

cv::Mat QImage2cvMat(QImage image) {
    cv::Mat mat;
//    qDebug() << image.format();
    switch (image.format()) {
        case QImage::Format_ARGB32:
        case QImage::Format_RGB32:
        case QImage::Format_ARGB32_Premultiplied:
            mat = cv::Mat(image.height(), image.width(), CV_8UC4, (void *) image.constBits(), image.bytesPerLine());
            break;
        case QImage::Format_RGB888:
            mat = cv::Mat(image.height(), image.width(), CV_8UC3, (void *) image.constBits(), image.bytesPerLine());
            cv::cvtColor(mat, mat, cv::COLOR_BGR2RGB);
            break;
        case QImage::Format_Indexed8:
            mat = cv::Mat(image.height(), image.width(), CV_8UC1, (void *) image.constBits(), image.bytesPerLine());
            break;
        default:
            // todo:
            break;
    }
    return mat;
}


void checkConfigsExist() {
    std::string folderPath = std::string(getenv("HOME")) + "/SytHmi";
//    std::cout << folderPath << std::endl;

    struct stat info;
    if (stat(folderPath.c_str(), &info) != 0) {
        // 文件夹不存在，创建它
        if (mkdir(folderPath.c_str(), 0777) == 0) {
            std::cout << "文件夹创建成功！" << std::endl;
        } else {
            std::cerr << "无法创建文件夹！" << std::endl;
            return;
        }
    } else if (info.st_mode & S_IFDIR) {
        // 文件夹已经存在
        std::cout << "文件夹已存在！" << std::endl;
    } else {
        std::cerr << "路径不是文件夹！" << std::endl;
        return;
    }

    std::string filePath = folderPath + "/config.yaml";
    struct stat info2;
    if (stat(filePath.c_str(), &info2) != 0) {
        cv::FileStorage fs(filePath, cv::FileStorage::WRITE);
        fs.release();
    } else if (info2.st_mode & S_IFREG) {
        // 文件已经存在
        std::cout << "配置文件已存在！" << std::endl;
    } else {
        std::cerr << "路径不是文件！" << std::endl;
        return;
    }


}

std::string getConfigPath() {
    std::string folderPath = std::string(getenv("HOME")) + "/SytHmi";
    std::string filePath = folderPath + "/config.yaml";
    return filePath;
};
