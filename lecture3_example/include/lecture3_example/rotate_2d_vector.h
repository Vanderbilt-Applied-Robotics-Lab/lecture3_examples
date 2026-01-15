#ifndef EIGEN_EXAMPLE_H
#define EIGEN_EXAMPLE_H

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <lecture3_example/msg/planar_vector.hpp> // note the snake case

/**
 * Rotates a 2D vector by specified angle
 * Applied Robotics
 * Author: Garrison Johnston
 */
class Rotate2dVector : public rclcpp::Node
{
public:
    /**
     * Constructor.
     * @param x Unrotated x component of vector
     * @param y Unrotated y component of vector
     * @param theta Angle to rotate vector by
     */
    Rotate2dVector(double x, double y, double theta);

    // destructor
    ~Rotate2dVector() = default;

    // publishes the rotated vector
    void publishRotatedVector();

private:
    /**
     * Create a 2D rotation matrix.
     * @param theta Angle of rotation matrix
     * @return The 2x2 rotation matrix
     */
    Eigen::Matrix2d createRotationMatrix(double theta);

    // publisher object
    rclcpp::Publisher<lecture3_example::msg::PlanarVector>::SharedPtr publisher_;

    // output message
    lecture3_example::msg::PlanarVector msg_ = lecture3_example::msg::PlanarVector();
    
    // Vector to rotate
    Eigen::Vector2d vector_; // this can also be written as Eigen::Vector<double,2> or  Eigen::Vector<double,2,1>

    // Time at start of node
    Eigen::Matrix2d rotation_matrix_; // this can also be written as Eigen::Matrix<double,2,2>
};

#endif // EIGEN_EXAMPLE_H