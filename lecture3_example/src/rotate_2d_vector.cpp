#include <lecture3_example/rotate_2d_vector.h>

Rotate2dVector::Rotate2dVector(double x, double y, double theta) : Node("rotate_2d_vector")
{
    publisher_ = this->create_publisher<lecture3_example::msg::PlanarVector>("/rotated_vector", 10); // 10 is history kept

    // assign x and y to vector
    vector_ << x, y;

    rotation_matrix_ = createRotationMatrix(theta);
}

void Rotate2dVector::publishRotatedVector()
{
    // multiply rotation matrix by vector
    Eigen::Vector2d rotated_vector = rotation_matrix_*vector_;

    // update header 
    msg_.header.stamp = this->get_clock()->now();
    msg_.header.frame_id = "world";

    msg_.x = rotated_vector(0);
    msg_.y = rotated_vector(1);

    RCLCPP_INFO(this->get_logger(), "Rotated vector: [%0.3f, %0.3f]", rotated_vector(0), rotated_vector(1));
    
    publisher_->publish(msg_);
}

Eigen::Matrix2d Rotate2dVector::createRotationMatrix(double theta)
{
    Eigen::Matrix2d rotation_matrix;

    rotation_matrix << std::cos(theta), -std::sin(theta), std::sin(theta), std::cos(theta);

    return rotation_matrix;
}

int main(int argc, char * argv[])
{
    // initialize the node
    rclcpp::init(argc, argv);

    // note: argv[0] is not a null pointer (or, equivalently, if argc > 0), it points to a string that represents the name used to invoke the program, or to an empty string.
    double x = std::stod(argv[1]); 
    double y = std::stod(argv[2]); 
    double theta = std::stod(argv[3]); 

    // create instance of class
    auto node = std::make_shared<Rotate2dVector>(x,y,theta);

    // MAIN LOOP
    node->publishRotatedVector();
    rclcpp::spin_some(node); // updates publishers and subscribers
    rclcpp::shutdown();
    return 0;
}
