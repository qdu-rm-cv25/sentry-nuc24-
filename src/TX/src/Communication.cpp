#include <node.hpp>
#include <fstream>
#include <endian.h>

int Communication::serial_init(std::string name, int baud)
{
    try
    {
        ser.setPort(name);
        ser.setBaudrate(baud);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        ROS_ERROR_STREAM("Unable to open port");
        return -1;
    }

    if (ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialied");
    }
    else
    {
        return -1;
    }
    return 1;
}

void Communication::vel_callback(const geometry_msgs::Twist &cmd_vel)
{
    if (__BYTE_ORDER == __LITTLE_ENDIAN)
    {
        Protocol_DownPackage_t packed;
        ROS_INFO("Receive a /cmd_vel message!");
        ROS_INFO("Linear Components:[%f,%f,%f]", cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.linear.z);
        ROS_INFO("Angular Componets:[%f,%f,%f]", cmd_vel.angular.x, cmd_vel.angular.y, cmd_vel.angular.z);

        memset(&packed, 0, sizeof(packed));

        packed.data.chassis_move_vec.vx = -cmd_vel.linear.y*0.03;
        packed.data.chassis_move_vec.vy = cmd_vel.linear.x*0.03;
        // packed.data.chassis_move_vec.vx = 0;
        // packed.data.chassis_move_vec.vy = 0.1;
        packed.data.chassis_move_vec.wz = 0;
        ROS_INFO("v:[%f,%f,%f]", packed.data.chassis_move_vec.vx, packed.data.chassis_move_vec.vy , packed.data.chassis_move_vec.wz);
        // packed.data.gimbal.yaw = yaw;
        packed.data.extern_channel.pitch = 0;
        packed.data.extern_channel.roll = 0;
        packed.data.extern_channel.yaw = 0;
        packed.data.extern_channel.x = 0;
        packed.data.extern_channel.y = 0;
        packed.data.extern_channel.z = 0;

        packed.data.notice = 0;
        

        // std::cout  << std::hex << packed.data.gimbal.yaw << std::endl;
        // std::cout  << std::hex << packed.data.gimbal.pit << std::endl;
        // std::cout  << std::hex << packed.data.gimbal.rol << std::endl;
        // std::cout  << std::hex << packed.data.notice << std::endl;
        // std::cout  << std::hex << packed.data.chassis_move_vec.vx << std::endl;
        // std::cout  << std::hex << packed.data.chassis_move_vec.vy << std::endl;
        // std::cout  << std::hex << packed.data.chassis_move_vec.wz << std::endl;
        // std::cout  << std::hex << packed.data.extern_channel.pitch << std::endl;
        // std::cout  << std::hex << packed.data.extern_channel.roll << std::endl;
        // std::cout  << std::hex <<packed.data.extern_channel.yaw<< std::endl;
        // std::cout  << std::hex <<packed.data.extern_channel.x<< std::endl;
        // std::cout  << std::hex <<packed.data.extern_channel.y<< std::endl;
        // std::cout  << std::hex <<packed.data.extern_channel.z<< std::endl;
        // std::stringstream ss;

        packed.crc16 = crc16::CRC16_Calc(reinterpret_cast<uint8_t*>(&packed),sizeof(packed)-sizeof(uint16_t),UINT16_MAX);
        //ROS_ERROR("[%f,%f,%f]",packed.data.chassis_move_vec.vx,packed.data.chassis_move_vec.vy,packed.data.chassis_move_vec.wz);
        ROS_ERROR("[%d]",packed.crc16);
        // std::ofstream ofs;
        // ofs.open("/home/rm/navdemo_ws/test1.bin", std::ios::app |std::ios::binary);

        // // ofs.write((const char *)reinterpret_cast<uint8_t*>(&packed), sizeof(uint8_t*));
        // //Integer data to write to binary file
        // // Write the data and close the file
        // ofs.write(reinterpret_cast<char *>(&packed), sizeof(packed));
        // ofs.close ();

        ser.write(reinterpret_cast<uint8_t*>(&packed),sizeof(packed));
        ser.flushOutput();
    }
}

void Communication::Odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}

Communication::Communication()
{
    pitch, roll, yaw = 0.0f;
    ros::NodeHandle n1;
    ros::Subscriber write_odom = n1.subscribe("odom", 1000, &Communication::Odom_callback, this);
    ros::Subscriber write_vel = n1.subscribe("cmd_vel", 1000, &Communication::vel_callback, this);
    ros::Publisher read_pub = n1.advertise<std_msgs::String>("read", 1000);
    if (serial_init("/dev/ttyACM0", 460800) == -1)
        return;
    ros::Rate loop_rate(200); // hz
    while (ros::ok())
    {

        ros::spinOnce();
        if (ser.available())
        {
            // ROS_INFO_STREAM("Reading from serial port");
            std_msgs::String result;
            result.data = ser.read(ser.available());
            // ROS_INFO_STREAM("Read:" << result.data);
            read_pub.publish(result);
        }
        loop_rate.sleep();
    }
}