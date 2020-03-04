#include "DashboardGUIPlugin.hh"

using namespace gazebo;

GZ_REGISTER_GUI_PLUGIN(DashboardGUIPlugin)

DashboardGUIPlugin::DashboardGUIPlugin()
  : GUIPlugin()
{
    initUI();
    initConnection();
}

DashboardGUIPlugin::~DashboardGUIPlugin()
{
}

void DashboardGUIPlugin::initUI()
{
    // Set the frame background and foreground colors
    this->setStyleSheet(
        "QFrame { background-color : rgba(100, 100, 100, 255); color : white; }");

    // Create the main layout
    QHBoxLayout *mainLayout = new QHBoxLayout;
    QFrame *mainFrame = new QFrame();
    QHBoxLayout *frameLayout = new QHBoxLayout();

    QLabel *altitudeTitle = new QLabel(tr("Altitude:"));
    QLabel *altitudeValue = new QLabel(tr(""));
    altitudeTitle->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
    altitudeTitle->setFixedSize(50,40);
    altitudeValue->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    altitudeValue->setFixedSize(50,40);



    QLabel *headingTitle = new QLabel(tr("Heading: "));
    QLabel *headingValue = new QLabel(tr(""));
    headingTitle->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    headingTitle->setFixedSize(50,40);
    headingValue->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    headingValue->setFixedSize(400,40);

    // Add the label to the frame's layout
    frameLayout->addWidget(altitudeTitle);
    frameLayout->addWidget(altitudeValue);
    connect(this, SIGNAL(setAltitude(QString)), altitudeValue, SLOT(setText(QString)), Qt::QueuedConnection);

    frameLayout->addWidget(headingTitle);
    frameLayout->addWidget(headingValue);
    connect(this, SIGNAL(setHeading(QString)), headingValue, SLOT(setText(QString)), Qt::QueuedConnection);

    // Add frameLayout to the frame
    mainFrame->setLayout(frameLayout);
    mainLayout->addWidget(mainFrame);

    // Remove margins to reduce space
    frameLayout->setContentsMargins(4, 4, 4, 4);
    mainLayout->setContentsMargins(0, 0, 0, 0);

    this->setLayout(mainLayout);

    // Position and resize this widget
    this->move(200, 10);
    this->resize(600, 40);
}

void DashboardGUIPlugin::initConnection()
{
    // Create a node for transportation
    this->node = transport::NodePtr(new transport::Node());
    this->node->Init("default");

    this->rangeSub = this->node->Subscribe("~/plane/link/lidar", &DashboardGUIPlugin::onRange, this);
    this->imuSub = this->node->Subscribe("~/plane/imu", &DashboardGUIPlugin::onIMU, this);
}

void DashboardGUIPlugin::onRange(RangePtr &_range)
{
    this->lastRangeMsg = *_range;
    this->setAltitude(QString::fromStdString(this->formatAltitude()));
}

void DashboardGUIPlugin::onIMU(IMUPtr &_imu)
{
    this->lastIMUMsg = *_imu;
    bodyQuaterniond = ignition::math::Quaterniond(
        lastIMUMsg.orientation().w(),
        lastIMUMsg.orientation().x(),
        lastIMUMsg.orientation().y(),
        lastIMUMsg.orientation().z());

    this->setHeading(QString::fromStdString(this->formatHeading()));
}

std::string DashboardGUIPlugin::formatAltitude()
{
    std::ostringstream stream;
    stream.str("");

    float altitude_m;
    altitude_m = this->lastRangeMsg.current_distance() * std::cos(this->bodyQuaterniond.Roll()) * std::cos(this->bodyQuaterniond.Pitch());

    stream << std::setw(2) << std::setfill('0') << altitude_m << "m";

    return stream.str();
}

std::string DashboardGUIPlugin::formatHeading()
{
    std::ostringstream stream;
    stream.str("");

    stream << std::setw(2) << std::setfill('0') << " Roll: " << floorf(this->bodyQuaterniond.Roll() * 360 / 6.28 * 100) / 100;
    stream << std::setw(2) << std::setfill('0') << " Pitch: " << floorf(this->bodyQuaterniond.Pitch() * 360 / 6.28 *100) / 100;
    stream << std::setw(2) << std::setfill('0') << " Yaw: " << floorf(this->bodyQuaterniond.Yaw() * 360 / 6.28 *100) / 100;
    return stream.str();
}