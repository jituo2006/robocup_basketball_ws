#pragma once
namespace geometry_msgs::msg {
struct Twist { struct Vector { double x=0,y=0,z=0; }; Vector linear,angular; };
}
