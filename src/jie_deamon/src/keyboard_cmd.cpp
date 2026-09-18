#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <stdio.h>
#include <termios.h>

static float linear_vel = 0.1;
static float angular_vel = 0.1;
static int k_vel = 3;

int GetCh()
{
  static struct termios oldt, newt;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  int c = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return c;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  printf("Keyboard Control for WPR Robot: \n");
  printf("w - Accelerate forward \n");
  printf("s - Accelerate backward \n");
  printf("a - Accelerate left \n");
  printf("d - Accelerate right \n");
  printf("q - Accelerate left rotation \n");
  printf("e - Accelerate right rotation \n");
  printf("space - Brake \n");
  printf("x - Exit \n");
  printf("------------- \n");

  auto node = rclcpp::Node::make_shared("keyboard_cmd");

  auto cmd_vel_pub = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  auto base_cmd = std::make_shared<geometry_msgs::msg::Twist>();
  base_cmd->linear.x = 0;
  base_cmd->linear.y = 0;
  base_cmd->angular.z = 0;

  while (rclcpp::ok())
  {
    int cKey = GetCh();
    if (cKey == 'w')
    {
      base_cmd->linear.x += linear_vel;
      if (base_cmd->linear.x > linear_vel * k_vel)
        base_cmd->linear.x = linear_vel * k_vel;
      cmd_vel_pub->publish(*base_cmd);
      printf(" - linear.x= %.2f linear.y= %.2f angular.z= %.2f \n", base_cmd->linear.x, base_cmd->linear.y, base_cmd->angular.z);
    }
    else if (cKey == 's')
    {
      base_cmd->linear.x += -linear_vel;

      if (base_cmd->linear.x < -linear_vel * k_vel)
        base_cmd->linear.x = -linear_vel * k_vel;
      cmd_vel_pub->publish(*base_cmd);
      printf(" - linear.x= %.2f linear.y= %.2f angular.z= %.2f \n", base_cmd->linear.x, base_cmd->linear.y, base_cmd->angular.z);
    }
    else if (cKey == 'a')
    {
      base_cmd->linear.y += linear_vel;
      if (base_cmd->linear.y > linear_vel * k_vel)
        base_cmd->linear.y = linear_vel * k_vel;
      cmd_vel_pub->publish(*base_cmd);
      printf(" - linear.x= %.2f linear.y= %.2f angular.z= %.2f \n", base_cmd->linear.x, base_cmd->linear.y, base_cmd->angular.z);
    }
    else if (cKey == 'd')
    {
      base_cmd->linear.y += -linear_vel;
      if (base_cmd->linear.y < -linear_vel * k_vel)
        base_cmd->linear.y = -linear_vel * k_vel;
      cmd_vel_pub->publish(*base_cmd);
      printf(" - linear.x= %.2f linear.y= %.2f angular.z= %.2f \n", base_cmd->linear.x, base_cmd->linear.y, base_cmd->angular.z);
    }
    else if (cKey == 'q')
    {
      base_cmd->angular.z += angular_vel;
      if (base_cmd->angular.z > angular_vel * k_vel)
        base_cmd->angular.z = angular_vel * k_vel;
      cmd_vel_pub->publish(*base_cmd);
      printf(" - linear.x= %.2f linear.y= %.2f angular.z= %.2f \n", base_cmd->linear.x, base_cmd->linear.y, base_cmd->angular.z);
    }
    else if (cKey == 'e')
    {
      base_cmd->angular.z += -angular_vel;
      if (base_cmd->angular.z < -angular_vel * k_vel)
        base_cmd->angular.z = -angular_vel * k_vel;
      cmd_vel_pub->publish(*base_cmd);
      printf(" - linear.x= %.2f linear.y= %.2f angular.z= %.2f \n", base_cmd->linear.x, base_cmd->linear.y, base_cmd->angular.z);
    }
    else if (cKey == ' ')
    {
      base_cmd->linear.x = 0;
      base_cmd->linear.y = 0;
      base_cmd->angular.z = 0;
      cmd_vel_pub->publish(*base_cmd);
      printf(" - linear.x= %.2f linear.y= %.2f angular.z= %.2f \n", base_cmd->linear.x, base_cmd->linear.y, base_cmd->angular.z);
    }
    else if (cKey == 'x')
    {
      base_cmd->linear.x = 0;
      base_cmd->linear.y = 0;
      base_cmd->angular.z = 0;
      cmd_vel_pub->publish(*base_cmd);
      printf(" - linear.x= %.2f linear.y= %.2f angular.z= %.2f \n", base_cmd->linear.x, base_cmd->linear.y, base_cmd->angular.z);
      printf("Exited! \n");
      return 0;
    }
    else
    {
      printf(" - Undefined command \n");
    }
  }

  rclcpp::shutdown();
  return 0;
}