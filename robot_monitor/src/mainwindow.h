#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "widget.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/header.hpp"
#include <QTimer>
#include <deque>
#include <thread>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_srvs/srv/empty.hpp"
#include <chrono>
QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE
class QLabel;
class MainWindow;
class Subscriber : public rclcpp::Node
{
public:
    Subscriber(MainWindow* wnd);
private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service_;
    MainWindow *mainWnd;
    std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> start_time;
};
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    struct Msg
    {
        double t;
        double q[28] = {0};
    };
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void pushMessage(double t, sensor_msgs::msg::JointState::SharedPtr msg);
    double getDataDuration();
    void clear();
    void log2file();
    void onTimer();
    void setJointDisplay();
    void setScaling(bool );
    void setWindowWidth();
    void setLineWidth();
    void zoomIn();
    void zoomOut();
    void zoomReset();
    void setLogging(bool checked);
private:
    Ui::MainWindow *ui;
    Widget *widgets[4];
    QLabel * message;
    unsigned int sampleCount = 50000;
    QList<QPointF> m_buffer[4][7];
    double time_width = 10;
    QTimer timer;
    std::vector<Msg> frames;
    bool isScaling;
    bool isLogging;
    std::shared_ptr<std::thread> tt;
    Subscriber::SharedPtr node;
    std::mutex mtx;
};
#endif // MAINWINDOW_H
