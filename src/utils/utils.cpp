#include "utils/utils.h"

QMap<QString, int> style_id_map = {
    std::map<QString, int>::value_type("低弹性", 10),
    std::map<QString, int>::value_type("中弹性", 11),
    std::map<QString, int>::value_type("高弹性", 12),
    std::map<QString, int>::value_type("低厚度", 20),
    std::map<QString, int>::value_type("中厚度", 21),
    std::map<QString, int>::value_type("高厚度", 22),
    std::map<QString, int>::value_type("2XS", 30),
    std::map<QString, int>::value_type("XS", 31),
    std::map<QString, int>::value_type("S", 32),
    std::map<QString, int>::value_type("M", 33),
    std::map<QString, int>::value_type("L", 34),
    std::map<QString, int>::value_type("XL", 35),
    std::map<QString, int>::value_type("2XL", 36),
    std::map<QString, int>::value_type("3XL", 37),
    std::map<QString, int>::value_type("4XL", 38),
    std::map<QString, int>::value_type("低光泽", 40),
    std::map<QString, int>::value_type("中光泽", 41),
    std::map<QString, int>::value_type("高光泽", 42),
    std::map<QString, int>::value_type("A", 60),
    std::map<QString, int>::value_type("B", 61),
    std::map<QString, int>::value_type("C", 62),
    std::map<QString, int>::value_type("D", 63),
};

QMap<int, QString> id_style_map = {
    std::map<int, QString>::value_type(10, "低弹性"),
    std::map<int, QString>::value_type(11, "中弹性"),
    std::map<int, QString>::value_type(12, "高弹性"),
    std::map<int, QString>::value_type(20, "低厚度"),
    std::map<int, QString>::value_type(21, "中厚度"),
    std::map<int, QString>::value_type(22, "高厚度"),
    std::map<int, QString>::value_type(30, "2XS"),
    std::map<int, QString>::value_type(31, "XS"),
    std::map<int, QString>::value_type(32, "S"),
    std::map<int, QString>::value_type(33, "M"),
    std::map<int, QString>::value_type(34, "L"),
    std::map<int, QString>::value_type(35, "XL"),
    std::map<int, QString>::value_type(36, "2XL"),
    std::map<int, QString>::value_type(37, "3XL"),
    std::map<int, QString>::value_type(38, "4XL"),
    std::map<int, QString>::value_type(40, "低光泽"),
    std::map<int, QString>::value_type(41, "中光泽"),
    std::map<int, QString>::value_type(42, "高光泽"),
    std::map<int, QString>::value_type(60, "A"),
    std::map<int, QString>::value_type(61, "B"),
    std::map<int, QString>::value_type(62, "C"),
    std::map<int, QString>::value_type(63, "D"),
};

int showMessageBox(QWidget *p, STATE state, QString text, int btn_num, QVector<QString> btn_text) {
  auto box = new QMessageBox(p);
  box->setWindowFlags(Qt::FramelessWindowHint | Qt::Dialog);
  box->setText(text);
  QImage qimage;
  switch (state) {
  case SUCCESS:
    qimage = QImage(":m_icon/icon/success.svg");
    break;
  case WARN:
    qimage = QImage(":m_icon/icon/warn.svg");
    break;
  case ERROR:
    qimage = QImage(":m_icon/icon/error.svg");
    break;
  }
  qimage = qimage.scaled(50, 50, Qt::AspectRatioMode::KeepAspectRatio, Qt::TransformationMode::SmoothTransformation);
  box->setIconPixmap(QPixmap::fromImage(qimage));
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
  qlonglong tick     = getTickCount();
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
    const uchar *pSrc = (const uchar *)mat.data;
    // Create QImage with same dimensions as input Mat
    QImage image(pSrc, mat.cols, mat.rows, mat.step, QImage::Format_RGB888);
    return image.rgbSwapped();
  } else if (mat.type() == CV_8UC4) {
    //        qDebug("CV_8UC4");
    // Copy input Mat
    const uchar *pSrc = (const uchar *)mat.data;
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
    mat = cv::Mat(image.height(), image.width(), CV_8UC4, (void *)image.constBits(), image.bytesPerLine());
    break;
  case QImage::Format_RGB888:
    mat = cv::Mat(image.height(), image.width(), CV_8UC3, (void *)image.constBits(), image.bytesPerLine());
    cv::cvtColor(mat, mat, cv::COLOR_BGR2RGB);
    break;
  case QImage::Format_Indexed8:
    mat = cv::Mat(image.height(), image.width(), CV_8UC1, (void *)image.constBits(), image.bytesPerLine());
    break;
  default:
    // todo: 如果存在其他的图像格式...
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
  std::string filePath   = folderPath + "/config.yaml";
  return filePath;
}

std::string getCurrentTime() {
  struct timeval tv;
  gettimeofday(&tv, NULL);

  static const int MAX_BUFFER_SIZE = 128;
  char timestamp_str[MAX_BUFFER_SIZE];
  time_t sec = static_cast<time_t>(tv.tv_sec);
  int ms     = static_cast<int>(tv.tv_usec) / 1000;

  struct tm tm_time;
  localtime_r(&sec, &tm_time);
  static const char *formater = "%4d-%02d-%02d_%02d:%02d:%02d.%03d";
  int wsize                   = snprintf(timestamp_str, MAX_BUFFER_SIZE, formater,
                                         tm_time.tm_year + 1900, tm_time.tm_mon + 1, tm_time.tm_mday,
                                         tm_time.tm_hour, tm_time.tm_min, tm_time.tm_sec, ms);

  timestamp_str[std::min(wsize, MAX_BUFFER_SIZE - 1)] = '\0';
  return std::string(timestamp_str);
}

void killProcesses(std::string process_pattern) {
  pid_t self_pid = getpid();
  FILE *pipe     = popen(("ps -x | grep -v grep | grep -i " + process_pattern + " | awk '{print $1}'").c_str(), "r");
  char buffer[128];
  while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
    int pid = atoi(buffer);
    if (pid != 0 && pid != self_pid) {
      int result = kill(pid, SIGKILL);
    }
  }
}
