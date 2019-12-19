#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <sstream>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <typeinfo>
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>


#define UNIX_SOCKET_PATH "/tmp/test.unixsocket"
#define SEND_SIZE_MAX 256

geometry_msgs::Twist joy_vel;

// レスポンスの値
enum
{
    OK,
    NG
};

int unix_client(const geometry_msgs::Twist::ConstPtr& input)
{
    int ret_code = 0;
    std::string output;
    std::ostringstream oss;

    oss << input->linear.x << ',' << input->linear.y << ',' << input->linear.z << ',' << 
    input->angular.x << ',' << input->angular.y << ',' << input->angular.z;

    output = oss.str();
    ROS_INFO("send data: %s", output.c_str());
    //const char *output_ptr = output.c_str();
    //printf("%s", typeid(output.c_str()).name());
    uint32_t buf_len = (uint32_t)output.length();
    //printf("buf_len = %u\n", buf_len);

    int fd = -1;
    ssize_t size = 0;
    int flags = 0; // MSG_WAITALLとかMSG_NOSIGNALをよく使うけど今回はサンプルなのでフラグは無し

    int response = -1; // レスポンス考えるの面倒だったので0:OK/1:NGで。

    // ソケットアドレス構造体
    struct sockaddr_un sun;
    memset(&sun, 0, sizeof(sun));

    // UNIXドメインのソケットを作成
    fd = socket(AF_LOCAL, SOCK_STREAM, 0);
    if (fd == -1)
    {
        printf("failed to create_socket(errno=%d:%s)\n", errno, strerror(errno));
        return -1;
    }

    // ソケットアドレス構造体に接続先(サーバー)を設定
    sun.sun_family = AF_LOCAL;
    strcpy(sun.sun_path, UNIX_SOCKET_PATH);

    // 上記設定を用いてサーバーに接続
    ret_code = connect(fd, (const struct sockaddr *)&sun, sizeof(sun));
    if (ret_code == -1)
    {
        printf("failed to create_socket(errno:%d, error_str:%s)\n", errno, strerror(errno));
        close(fd);
        return -1;
    }

    // 送信内容の作成(ここは自由)
    //printf("please input what you want to send...\n>");
    //if (fgets(output.c_str(), SEND_SIZE_MAX, stdin) == NULL)
    //{
    //    printf("failed to fgets\n");
    //    close(fd);
    //    return -1;
    //}

    // データサイズの送信(ヘッダーの構造とか考えなきゃだめだけど今回は省略。データサイズのみ送る)
    size = send(fd, &buf_len, sizeof(buf_len), flags);
    if (size < sizeof(buf_len))
    {
        // partial sendパターンも今回はエラーとする
        printf("failed to send data size(errno:%d, error_str:%s)\n", errno, strerror(errno));
        close(fd);
        return -1;
    }

    // データ本体の送信
    size = send(fd, output.c_str(), buf_len, flags);
    if (size < buf_len)
    {
        // partial sendパターンも今回はエラーとする
        printf("failed to send data(errno:%d, error_str:%s)\n", errno, strerror(errno));
        close(fd);
        return -1;
    }

    // レスポンスの受信(今回は0:OK/1:NG)
    size = recv(fd, &response, sizeof(response), flags);
    if (size < sizeof(response))
    {
        // partial recvパターンも今回はエラーとする
        printf("failed to recv response(errno:%d, error_str:%s)\n", errno, strerror(errno));
        close(fd);
        return -1;
    }

    printf("respose = %d\n", response);

    // ソケットを閉じる
    close(fd);

    return 0;
}

void teleop_callback(const geometry_msgs::Twist::ConstPtr& input)
{
    // クライアント処理を実行
    int ret_code = 0;
    ret_code = unix_client(input);
    if (ret_code != 0)
    {
        ROS_ERROR("return value from client is invalid\n");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "socket_teleop_subscriber");
    ros::NodeHandle n;

    ros::Subscriber teleop_sub = n.subscribe("cmd_vel", 10, teleop_callback);
    ros::Rate loop_rate(10);

    while (ros::ok)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return EXIT_SUCCESS;
}
