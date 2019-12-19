#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/types.h>
#include <iostream>
#include <errno.h>
#include <string>
#include <typeinfo>
#include <sys/socket.h>
#include <sys/un.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>

#define UNIX_SOCKET_PATH "/tmp/test.unixsocket"

enum
{
    OK,
    NG
};

int unix_client(int32_t const *input)
{
    //std::cout << "data = " << *input << std::endl;

    uint32_t buf_len;
    int ret_code = 0;

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
    //buf_len = std::to_string(input).length();
    int digit = 0, num = *input;
    while (num != 0)
    {
        num = num / 10;
        ++digit;
    }
    buf_len = digit;
    //printf("buf_len = %u\n", buf_len);

    // データサイズの送信(ヘッダーの構造とか考えなきゃだめだけど今回は省略。データサイズのみ送る)
    size = send(fd, &buf_len, sizeof(buf_len), flags);
    if (size < buf_len)
    {
        //partial sendパターンも今回はエラーとする
        printf("failed to send data size(errno:%d, error_str:%s)\n", errno, strerror(errno));
        close(fd);
        return -1;
    }

    // データ本体の送信
    size = send(fd, input, buf_len, flags);
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

    printf("response = %d\n", response);

    // ソケットを閉じる
    close(fd);

    return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "socket_sample");
    ros::NodeHandle n;
    ros::Publisher int_pub = n.advertise<std_msgs::Int32>("socket_int", 1000);
    ros::Rate loop_rate(10);

    int32_t count = 0;
    std_msgs::Int32 msg;
    while (ros::ok())
    {
        count++;
        msg.data = count;
        ROS_INFO("%d", msg.data);
        //std::cout << "msg_ptr = " << int32_t(msg.data) << std::endl;
        //std::cout << "msg_ptr type = " << typeid(msg.data).name() << std::endl;
        int_pub.publish(msg);
        unix_client(&count);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}