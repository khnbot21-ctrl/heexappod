#include <cmath>
#include <chrono>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

static int lut_walk_0_length = 28;
static int lut_walk_0[28][6][3] = {{{350, 529, 448},
                                   {350, 275, 312},
                                   {350, 171, 252},
                                   {350, 275, 312},
                                   {350, 529, 448},
                                   {350, 425, 388}},
                                  {{357, 515, 433},
                                   {343, 275, 313},
                                   {358, 164, 240},
                                   {355, 273, 306},
                                   {340, 525, 446},
                                   {355, 422, 380}},
                                  {{363, 499, 415},
                                   {337, 275, 313},
                                   {366, 163, 232},
                                   {360, 270, 299},
                                   {330, 516, 441},
                                   {359, 419, 372}},
                                  {{368, 479, 397},
                                   {330, 276, 315},
                                   {374, 172, 230},
                                   {366, 269, 293},
                                   {321, 502, 432},
                                   {363, 415, 364}},
                                  {{372, 459, 378},
                                   {324, 276, 316},
                                   {381, 189, 234},
                                   {371, 267, 288},
                                   {314, 484, 420},
                                   {367, 411, 355}},
                                  {{374, 438, 359},
                                   {317, 277, 318},
                                   {387, 212, 243},
                                   {378, 266, 283},
                                   {309, 464, 407},
                                   {370, 407, 345}},
                                  {{376, 417, 341},
                                   {311, 278, 321},
                                   {390, 238, 257},
                                   {384, 265, 278},
                                   {306, 442, 392},
                                   {373, 402, 335}},
                                  {{376, 396, 324},
                                   {305, 280, 324},
                                   {392, 264, 274},
                                   {392, 264, 274},
                                   {305, 420, 376},
                                   {376, 396, 324}},
                                  {{373, 402, 335},
                                   {306, 258, 308},
                                   {384, 265, 278},
                                   {390, 238, 257},
                                   {311, 422, 379},
                                   {376, 417, 341}},
                                  {{370, 407, 345},
                                   {309, 236, 293},
                                   {378, 266, 283},
                                   {387, 212, 243},
                                   {317, 423, 382},
                                   {374, 438, 359}},
                                  {{367, 411, 355},
                                   {314, 216, 280},
                                   {371, 267, 288},
                                   {381, 189, 234},
                                   {324, 424, 384},
                                   {372, 459, 378}},
                                  {{363, 415, 364},
                                   {321, 198, 268},
                                   {366, 269, 293},
                                   {374, 172, 230},
                                   {330, 424, 385},
                                   {368, 479, 397}},
                                  {{359, 419, 372},
                                   {330, 184, 259},
                                   {360, 270, 299},
                                   {366, 163, 232},
                                   {337, 425, 387},
                                   {363, 499, 415}},
                                  {{355, 422, 380},
                                   {340, 175, 254},
                                   {355, 273, 306},
                                   {358, 164, 240},
                                   {343, 425, 387},
                                   {357, 515, 433}},
                                  {{350, 425, 388},
                                   {350, 171, 252},
                                   {350, 275, 312},
                                   {350, 171, 252},
                                   {350, 425, 388},
                                   {350, 529, 448}},
                                  {{345, 427, 394},
                                   {360, 175, 254},
                                   {345, 278, 320},
                                   {343, 185, 267},
                                   {357, 425, 387},
                                   {342, 536, 460}},
                                  {{340, 430, 401},
                                   {370, 184, 259},
                                   {341, 281, 328},
                                   {337, 201, 285},
                                   {363, 425, 387},
                                   {334, 537, 468}},
                                  {{334, 431, 407},
                                   {379, 198, 268},
                                   {337, 285, 336},
                                   {332, 221, 303},
                                   {370, 424, 385},
                                   {326, 528, 470}},
                                  {{329, 433, 412},
                                   {386, 216, 280},
                                   {333, 289, 345},
                                   {328, 241, 322},
                                   {376, 424, 384},
                                   {319, 511, 466}},
                                  {{322, 434, 417},
                                   {391, 236, 293},
                                   {330, 293, 355},
                                   {326, 262, 341},
                                   {383, 423, 382},
                                   {313, 488, 457}},
                                  {{316, 435, 422},
                                   {394, 258, 308},
                                   {327, 298, 365},
                                   {324, 283, 359},
                                   {389, 422, 379},
                                   {310, 462, 443}},
                                  {{308, 436, 426},
                                   {395, 280, 324},
                                   {324, 304, 376},
                                   {324, 304, 376},
                                   {395, 420, 376},
                                   {308, 436, 426}},
                                  {{310, 462, 443},
                                   {389, 278, 321},
                                   {324, 283, 359},
                                   {327, 298, 365},
                                   {394, 442, 392},
                                   {316, 435, 422}},
                                  {{313, 488, 457},
                                   {383, 277, 318},
                                   {326, 262, 341},
                                   {330, 293, 355},
                                   {391, 464, 407},
                                   {322, 434, 417}},
                                  {{319, 511, 466},
                                   {376, 276, 316},
                                   {328, 241, 322},
                                   {333, 289, 345},
                                   {386, 484, 420},
                                   {329, 433, 412}},
                                  {{326, 528, 470},
                                   {370, 276, 315},
                                   {332, 221, 303},
                                   {337, 285, 336},
                                   {379, 502, 432},
                                   {334, 431, 407}},
                                  {{334, 537, 468},
                                   {363, 275, 313},
                                   {337, 201, 285},
                                   {341, 281, 328},
                                   {370, 516, 441},
                                   {340, 430, 401}},
                                  {{342, 536, 460},
                                   {357, 275, 313},
                                   {343, 185, 267},
                                   {345, 278, 320},
                                   {360, 525, 446},
                                   {345, 427, 394}}};

class LUTPlayer : public rclcpp::Node
{
public:
    LUTPlayer() : Node("lut_player")
    {
        // Untuk RViz
        joint_pub_ =
            create_publisher<sensor_msgs::msg::JointState>(
                "/joint_states", 10);

        // Untuk Gazebo ros2_control
        cmd_pub_ =
            create_publisher<std_msgs::msg::Float64MultiArray>(
                "/position_controller/commands", 10);

        timer_ =
            create_wall_timer(
                std::chrono::milliseconds(12),
                std::bind(&LUTPlayer::update, this));

        RCLCPP_INFO(get_logger(), "LUT Player Started");
    }

private:

    double tickToRad(int tick)
    {
        double deg = (tick - 350) * 0.4;
        return deg * M_PI / 180.0;
    }

    void update()
    {
        sensor_msgs::msg::JointState js_msg;

        js_msg.header.stamp = now();

        js_msg.name =
        {
            "cdk_joint",
            "fdk_joint",
            "tdk_joint",

            "ctk_joint",
            "ftk_joint",
            "ttk_joint",

            "cbk_joint",
            "fbk_joint",
            "tbk_joint",

            "cdr_joint",
            "fdr_joint",
            "tdr_joint",

            "ctr_joint",
            "ftr_joint",
            "ttr_joint",

            "cbr_joint",
            "fbr_joint",
            "tbr_joint"
        };

        const size_t f = frame_idx_;

        js_msg.position =
        {
            // LEFT FRONT
            tickToRad(lut_walk_0[f][3][0]),
            -tickToRad(lut_walk_0[f][3][1]),
            tickToRad(lut_walk_0[f][3][2]),

            // LEFT MIDDLE
            tickToRad(lut_walk_0[f][4][0]),
            -tickToRad(lut_walk_0[f][4][1]),
            tickToRad(lut_walk_0[f][4][2]),

            // LEFT REAR
            tickToRad(lut_walk_0[f][5][0]),
            -tickToRad(lut_walk_0[f][5][1]),
            tickToRad(lut_walk_0[f][5][2]),

            // RIGHT FRONT
            tickToRad(lut_walk_0[f][0][0]),
            -tickToRad(lut_walk_0[f][0][1]),
            tickToRad(lut_walk_0[f][0][2]),

            // RIGHT MIDDLE
            tickToRad(lut_walk_0[f][1][0]),
            -tickToRad(lut_walk_0[f][1][1]),
            tickToRad(lut_walk_0[f][1][2]),

            // RIGHT REAR
            tickToRad(lut_walk_0[f][2][0]),
            -tickToRad(lut_walk_0[f][2][1]),
            tickToRad(lut_walk_0[f][2][2])
        };

        // ===============================
        // Publish ke Gazebo
        // ===============================

        std_msgs::msg::Float64MultiArray gz_msg;

        gz_msg.data = js_msg.position;

        // Publish ke RViz
        joint_pub_->publish(js_msg);

        // Publish ke Gazebo
        cmd_pub_->publish(gz_msg);

        frame_idx_++;
        frame_idx_ %= lut_walk_0_length;
    }

    size_t frame_idx_ = 0;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
};

// ======================================================

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<LUTPlayer>());

    rclcpp::shutdown();

    return 0;
}