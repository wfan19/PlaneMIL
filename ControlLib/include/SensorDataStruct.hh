class SensorDataStruct{

public:
    // Struct containing sensor data
    // This struct actually belongs with your sensor drivers. However as I don't really have any I'll just leave this here.
    struct SensorData
    {
        double range;

        double pitch; // assumes up is positive
        double roll; // assumes cw is positive (looking from behind)
        // double yaw;
    };
};