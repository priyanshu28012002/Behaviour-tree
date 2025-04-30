// ========== PrintBatteryStatus Node ==========
class PrintBatteryStatus : public BT::SyncActionNode
{
public:
    PrintBatteryStatus(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<float>("battery_status") };
    }

    BT::NodeStatus tick() override
    {
        float value;
        if (!getInput("battery_status", value)) {
            std::cerr << "PrintBatteryStatus: Failed to get battery_status from blackboard\n";
            return BT::NodeStatus::FAILURE;
        }

        std::cout << "[PrintBatteryStatus] Battery level: " << value << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};

// ========== PrintRisk Node ==========
class PrintRisk : public BT::SyncActionNode
{
public:
    PrintRisk(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<std::vector<float>>("risk") };
    }

    BT::NodeStatus tick() override
    {
        std::vector<float> risk;
        if (!getInput("risk", risk)) {
            std::cerr << "PrintRisk: Failed to get risk from blackboard\n";
            return BT::NodeStatus::FAILURE;
        }

        std::cout << "[PrintRisk] Risk values: ";
        for (float v : risk) {
            std::cout << v << " ";
        }
        std::cout << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};

// ========== PrintOdom Node ==========
class PrintOdom : public BT::SyncActionNode
{
public:
    PrintOdom(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::vector<double>>("position"),
            BT::InputPort<std::vector<double>>("orientation")
        };
    }

    BT::NodeStatus tick() override
    {
        std::vector<double> position, orientation;

        if (!getInput("position", position)) {
            std::cerr << "[PrintOdom] Failed to get position from blackboard\n";
            return BT::NodeStatus::FAILURE;
        }

        if (!getInput("orientation", orientation)) {
            std::cerr << "[PrintOdom] Failed to get orientation from blackboard\n";
            return BT::NodeStatus::FAILURE;
        }

        std::cout << "[PrintOdom] Position: ";
        for (double p : position) std::cout << p << " ";
        std::cout << "\n[PrintOdom] Orientation: ";
        for (double o : orientation) std::cout << o << " ";
        std::cout << std::endl;

        return BT::NodeStatus::SUCCESS;
    }
};

